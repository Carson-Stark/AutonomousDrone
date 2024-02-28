#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from ObjectDetection import find_bag
from BlobDetection import detectWhite
from cv_bridge import CvBridge
import cv2
import numpy as np
from dronekit import connect, VehicleMode, mavutil, LocationGlobal, LocationGlobalRelative, Command, SystemStatus
import time
import math
from drone.srv import Coordinates
import pyrealsense2 as rs2
import offboard_utils
from mission_utils import MissionItem, CommandType, add_search_waypoints
from std_msgs.msg import String
from datetime import datetime

vehicle = None
front_depth = None
down_frame = None
down_depth = None
cameraInfo = None
last_frame_update = 0
        
def check_for_bag (frame):
    if vehicle == None or frame is None:
        print("no frame")
        return None
    if time.time() - last_frame_update > 1:
        print("time since last frame " + str(time.time() - last_frame_update))
        return None

    alt = vehicle.location.global_relative_frame.alt
    pixel_size = offboard_utils.world_to_pixel((0.5, 0.5, alt), cameraInfo) #max pixel size of bag
    detection = None
    if vehicle.rangefinder.distance < 0.8:
        detection = detectWhite(frame, pixel_size[0] * pixel_size[1])
    
    detection = find_bag(frame)

    return detection

    """best = None
    for detection in detections:
        if detection[0] == 0 and detection[1] > 0.6 and (best == None or detection[1] > best[1]) and is_valid_detection(detection, frame):
            best = detection
        
    return best"""

# (horizontal, vertical)
def get_bag_image_coords ():
    detection = check_for_bag(down_frame)
    if detection == None:
        return None
    
    pixel_offsets = offboard_utils.get_offsets(vehicle, down_frame, cameraInfo)
    #print(down_frame.shape)
    #print(detection)
    x0, x1, y0, y1 = detection
    width = x1 - x0
    height = y1 - y0
    centerx = (x0 + x1) // 2 - (down_frame.shape[1] / 2)
    centery = (y0 + y1) // 2 - (down_frame.shape[0] / 2)
    #print (f"{centerx} {centery} {width} {height}")
    if centerx > 0:
        centered_x = x0 + (width // 4)  - (down_frame.shape[1] / 2)
    else:
        centered_x = x0 + (width // 4 * 3)  - (down_frame.shape[1] / 2)
    if centery > 0:
        centered_y = y0 + (height // 4)  - (down_frame.shape[0] / 2)
    else:
        centered_y = y0 + (height // 4 * 3)  - (down_frame.shape[0] / 2)
    #centered_y = y1 - down_frame.shape[0] / 2

    coordinates = ((centered_x + pixel_offsets[0]) / down_frame.shape[1], \
                   (centered_y + pixel_offsets[1]) / -down_frame.shape[0])
    pixel_x = centerx + pixel_offsets[0]
    pixel_y = centery + pixel_offsets[1]
    return (coordinates, (pixel_x, pixel_y))

def search_for_bag (rate):
    vehicle.simple_goto(vehicle.location.global_frame)
    while not rospy.is_shutdown() and vehicle.armed:
        if check_for_bag(down_frame) is not None:
            land_on_object(rate)
        rate.sleep()

def estimate_velocity(obj_pos_history):
    if len(obj_pos_history) <= 5:
        return
    
    pos1 = obj_pos_history[-5]
    pos2 = obj_pos_history[-1] #skip one frame

    #don't estimate velocity if turning
    if math.degrees(abs(pos1[3] - pos2[3])) > 5:
        return

    time_diff = pos2[4] - pos1[4]
    velocity_vector = ((pos2[0] - pos1[0]) / time_diff, (pos2[1] - pos1[1]) / time_diff, (pos2[2] - pos1[2]) / time_diff)

    print("Estimated: " + str((velocity_vector[1], velocity_vector[0], velocity_vector[2])))
    print("True: " + str(vehicle.velocity))
    msg = vehicle.message_factory.vision_speed_estimate_encode(
        0, #time boot
        velocity_vector[1], velocity_vector[0], velocity_vector[2],
        [float("NAN"), 0, 0, 0, 0, 0, 0, 0, 0], #covariance
        0, # reset counter
    )
    #vehicle.send_mavlink(msg)

    return velocity_vector

def check_obstacles(min_dist):
    if vehicle.location.global_relative_frame.alt < 1:
        return False

    for c in range(100, front_depth.shape[1] - 100, 20):
        count = 0
        for r in range(100, front_depth.shape[0] - 100, 10):
            dist_m = front_depth[r, c] * 0.001
            if dist_m != 0 and dist_m < min_dist:
                count += 1
        if count > 3:
            return True

    return False

def check_downward_obs(min_dis=3):
    if vehicle.location.global_relative_frame.alt < min_dis * 1.4:
        return -1

    top = 0
    right = 0
    bottom = 0
    left = 0
    obstacle = False
    for r in range(0, down_depth.shape[0])[::20]:
        count = 0
        for c in range(0, down_depth.shape[1])[::10]:
            depth = down_depth[r][c] * 0.001
            if depth == 0:
                depth = 10
            elif depth < min_dis and depth > 0.8 and depth != 0:
                count += 1

            if r < down_depth.shape[0] * 0.33:
                top += depth
            if c > down_depth.shape[1] * 0.66:
                right += depth
            if r > down_depth.shape[0] * 0.66:
                bottom += depth
            if c < down_depth.shape[1] * 0.33:
                left += depth
        if count > 3:
            obstacle = True

    if not obstacle:
        return -1

    top = top / (down_depth.shape[0] * down_depth.shape[1]) / 100
    right = right / (down_depth.shape[0] * down_depth.shape[1]) / 100
    bottom = bottom / (down_depth.shape[0] * down_depth.shape[1]) / 100
    left = left / (down_depth.shape[0] * down_depth.shape[1]) / 100

    directions = [top, right, bottom, left]
    best_dir = np.argmin(directions)
    
    return best_dir

def check_safe_landing(num_bins=5, bin_size=50):
    bins = [0] * (num_bins * num_bins)
    startr = int(down_depth.shape[0]/2 - (num_bins/2.0) * bin_size)
    startc = int(down_depth.shape[1]/2 - (num_bins/2.0) * bin_size)
    for r in range(startr, down_depth.shape[0] - startr)[::10]:
        for c in range(startc, down_depth.shape[1] - startc)[::10]:
            binr = int((r - startr) / bin_size)
            binc = int((c - startc) / bin_size)
            bin = binr * num_bins + binc
            bins[bin] += down_depth[r][c] * 0.001

    for b in range(len(bins)):
        bins[b] = bins[b] / math.pow(bin_size / 10, 2)

    avg_depth = sum(bins) / len(bins)
    print(avg_depth)
    avg_deviation = sum(abs(d - avg_depth) for d in bins) / len(bins)
    print(("SAFE: " if avg_deviation < 0.1 else "UNSAFE: ") + str(avg_deviation))
    return avg_deviation < 0.1


def try_land(rate):
    yaw = vehicle.attitude.yaw
    while check_safe_landing():
        print("trying safe landing")
        alt = vehicle.rangefinder.distance
        offboard_utils.send_frame_velocity(vehicle, 0, 0, 1, yaw)
        if alt < 1.5:
            vehicle.mode = VehicleMode("LAND")
            return True
        rate.sleep()
    return False

def check_for_payload():    
    close_points = 0
    for r in range(0, down_depth.shape[0], 10):
        for c in range(0, down_depth.shape[1], 10):
            depth = down_depth[r][c] * 0.001
            if depth < 0.5 and depth != 0:
                close_points += 1

    print (close_points)
    return close_points > 25

def clamp(n, mag): 
    if n < -mag: 
        return -mag
    elif n > mag: 
        return mag
    else: 
        return n

def land_on_object(rate):

    yaw = vehicle.attitude.yaw
    start_pos = vehicle.location.global_frame
    start_alt = vehicle.rangefinder.distance
    relative_coordinates = None

    print ("landing on object")
    tolerance = 0.25

    def within_tolerance(multiplier=1):
        return abs(relative_coordinates[0]) < tolerance * multiplier and abs(relative_coordinates[1]) < tolerance * multiplier

    pos_buff = []

    #center bag and maintain altitude
    final_descent = False
    final_descent_height = 0.6
    centered_frames = 0
    centered = False
    lost_for = 0
    while not final_descent and in_autonomous_mode():
        alt = vehicle.rangefinder.distance
        object_coordinates = get_bag_image_coords()
        if object_coordinates == None:
            print("target lost")
            lost_for += 1
            if lost_for > 2:
                offboard_utils.send_frame_velocity(vehicle, 0, 0, -0.2, yaw)
                centered_frames = 0
            if alt > start_alt + 0.5 or lost_for > 10:
                break
            #vehicle.airspeed = 1
            #vehicle.simple_goto(start_pos)
            rate.sleep()
            continue

        lost_for = 0

        pixel_coordinates = object_coordinates[1]
        relative_coordinates = object_coordinates[0]
        local_coordinates = offboard_utils.pixel_to_world((pixel_coordinates[0], pixel_coordinates[1], alt), cameraInfo)
        world_coordinates = offboard_utils.rotate_to_global(vehicle, (local_coordinates[0], local_coordinates[1]))
        pos_buff.append((world_coordinates[0], world_coordinates[1], alt, vehicle.attitude.yaw, time.time()))
        if len(pos_buff) > 6:
            pos_buff.pop(0)
        estimate_velocity(pos_buff)

        if (within_tolerance() and alt < final_descent_height and centered_frames >= 3) or alt < 0.4:
            final_descent = True
            print ("final descent")
            break

        adjustment_speed = 1 if alt > 1.5 else 0.75
        if within_tolerance(0.5):
            adjustment_speed = 0

        max = 0.5

        descent_speed = 0.3 if alt > 1 else 0.2
        if alt < final_descent_height:
            descent_speed = 0
        if alt < final_descent_height * 0.8:
            descent_speed = -0.2

        x_speed = clamp(relative_coordinates[0] * adjustment_speed, max)
        y_speed = clamp(relative_coordinates[1] * adjustment_speed, max)

        if not within_tolerance():
            centered_frames = 0
            centered = False
            print ("uncentered: " + str(alt) + " " + str(relative_coordinates))
            offboard_utils.send_frame_velocity (vehicle, y_speed, x_speed, -0.2 if alt < 1 else 0, yaw)
        else:

            #if alt < final_descent_height:
            centered_frames += 1

            print ("centered: " + str(alt) + " " + str(relative_coordinates))
            offboard_utils.send_frame_velocity (vehicle, y_speed, x_speed, descent_speed, yaw)

        rate.sleep()

    if not final_descent:
        print ("landing attempt failed")
        vehicle.airspeed = 1
        vehicle.simple_goto(start_pos)
        return False

    offboard_utils.set_gripper(vehicle, False)
    offboard_utils.send_velocity_for(vehicle, 0.2, 0, 0, 0.5)
    offboard_utils.send_velocity_for(vehicle, 0, 0, 1, 4)
    vehicle.mode = VehicleMode("LAND")
    time.sleep(5)
    offboard_utils.set_gripper(vehicle, True)
    time.sleep(2)
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)
    return True 

started_mission = False
def bag_mission(target):
    x, y = target

    mission_alt = 20
    search_alt = 5
    print ("Starting bag pickup mission")

    cmds = []
    launch_pos = vehicle.location.global_relative_frame

    target = LocationGlobalRelative(x, y, mission_alt)
    distxy = offboard_utils.distance_meters(vehicle, target)
    dist = math.sqrt(pow(distxy[0], 2) + pow(distxy[1], 2))
    log(f"Target is {dist}m  meters away\n")
    if dist > 1000:
        log("Target is too far away, aborting mission")
        return
    
    if not vehicle.is_armable:
        log("Not armable, aborting mission")
        return

    if time.time() - last_frame_update > 1:
        log("Camera frame not being updated")
        return

    home = LocationGlobalRelative(launch_pos.lat, launch_pos.lon, mission_alt)
    cmds.append(MissionItem(CommandType.TAKEOFF, mission_alt, "Takeoff to mission altitude"))
    cmds.append(MissionItem(CommandType.WAYPOINT, home, "Start"))
    cmds.append(MissionItem(CommandType.WAYPOINT, target, "Fly to bag"))
    target_search = LocationGlobalRelative(x, y, search_alt)
    cmds.append(MissionItem(CommandType.WAYPOINT, target_search, "Descend to search altitude"))
    add_search_waypoints(cmds, target_search, 4, 1, search_alt, 0.5, "Search for bag", True)
    cmds.append(MissionItem(CommandType.PICKUP_PAYLOAD, 2, "Pickup Payload"))
    cmds.append(MissionItem(CommandType.TAKEOFF, mission_alt, "Ascend to mission altitude"))
    cmds.append(MissionItem(CommandType.WAYPOINT, home, "Fly to home"))
    home_land = LocationGlobalRelative(launch_pos.lat, launch_pos.lon, search_alt)
    cmds.append(MissionItem(CommandType.WAYPOINT, home_land, "Descend to search altitude"))
    add_search_waypoints(cmds, home, 4, 1, search_alt, 0.5, "Search for safe landing", False)
    cmds.append(MissionItem(CommandType.FINISH, launch_pos, "Finish"))

    do_mission("Pickup Poop", cmds)

def summon_mission(target):
    x, y = target

    mission_alt = 10    
    search_alt = 4
    log ("Starting summon mission")
    
    target = LocationGlobalRelative(x, y, mission_alt)
    distxy = offboard_utils.distance_meters(vehicle, target)
    dist = math.sqrt(pow(distxy[0], 2) + pow(distxy[1], 2))
    log(f"Target is {dist}m  meters away\n")
    if dist > 1000:
        log("Target is too far away, aborting mission")
        return

    if not vehicle.is_armable:
        log("Not armable, aborting mission")
        return

    if time.time() - last_frame_update > 1:
        log("Camera frame not being updated")
        return
    
    cmds = []
    launch_pos = vehicle.location.global_relative_frame
    cmds.append(MissionItem(CommandType.TAKEOFF, mission_alt, "Takeoff to mission altitude"))
    home = LocationGlobalRelative(launch_pos.lat, launch_pos.lon, search_alt)
    cmds.append(MissionItem(CommandType.WAYPOINT, home, "Start"))
    cmds.append(MissionItem(CommandType.WAYPOINT, target, "Fly to target"))
    target_search = LocationGlobalRelative(x, y, search_alt)
    cmds.append(MissionItem(CommandType.WAYPOINT, target_search, "Descend to search altitude"))
    add_search_waypoints(cmds, target_search, 4, 1, search_alt, 0.5, "Search for safe landing", False)
    cmds.append(MissionItem(CommandType.FINISH, launch_pos, "Finish"))

    do_mission("Go to Target", cmds)

def bag_land_mission():
    mission_alt = 10
    search_alt = 4
    log ("Starting land on bag mission")

    if not vehicle.is_armable:
        log("Not armable, aborting mission")
        return

    if time.time() - last_frame_update > 1:
        log("Camera frame not being updated")
        return

    cmds = []
    launch_pos = vehicle.location.global_relative_frame
    cmds.append(MissionItem(CommandType.TAKEOFF, search_alt, "Takeoff to search altitude"))
    home = LocationGlobalRelative(launch_pos.lat, launch_pos.lon, search_alt)
    cmds.append(MissionItem(CommandType.WAYPOINT, home, "Start"))
    target_search = LocationGlobalRelative(launch_pos.lat, launch_pos.lon, search_alt)
    add_search_waypoints(cmds, target_search, 2, 2, search_alt, 0.5, "Search for bag", True)
    cmds.append(MissionItem(CommandType.PICKUP_PAYLOAD, 2, "Takeoff with bag"))
    cmds.append(MissionItem(CommandType.GRIPPER, False, "Release Gripper"))
    cmds.append(MissionItem(CommandType.FINISH, target_search, "Finish"))

    do_mission("Search for Bag", cmds)

def start_safe_land():
    global started_mission

    vehicle.mode = VehicleMode("LOITER")
    started_mission = False

    search_alt = 4
    log ("Starting safe land")

    if not vehicle.is_armable:
        log("Not armable, aborting mission")
        return

    if time.time() - last_frame_update > 1:
        log("Camera frame not being updated")
        return

    cmds = []
    launch_pos = vehicle.location.global_relative_frame
    target_search = LocationGlobalRelative(launch_pos.lat, launch_pos.lon, search_alt)
    cmds.append(MissionItem(CommandType.WAYPOINT, target_search, "Descend to search altitude"))
    add_search_waypoints(cmds, target_search, 4, 1, search_alt, 0.5, "Search for safe landing", False)
    cmds.append(MissionItem(CommandType.FINISH, launch_pos, "Finish"))

    do_mission("Safe Land", cmds)

def send_mission_status(name, start, item_start, total_items, item1, item2, item3):
    msg = "Mission:\n"
    msg += name + "\n"
    msg += str(total_items) + "\n"
    msg += start + "\n"
    msg += item_start + "\n"
    msg += item1 + "\n"
    msg += item2 + "\n"
    msg += item3 + "\n"
    msg_pub.publish (msg)

def get_item_title(items, index):
    if index < len(items):
        return items[index].title
    return "-"

def do_mission(mission_title, cmd_items):
    global started_mission

    if started_mission:
        log("mission already running")
        return

    log("uploading mission...")
    vehicle.commands.clear()
    print (len(cmd_items))
    for item in cmd_items:
        item.add_commands(vehicle)
    time.sleep(1)
    vehicle.commands.upload()

    for cmd in vehicle.commands:
        print(cmd.x)

    # pre-arm checks
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    if not vehicle.is_armable or not vehicle.mode == VehicleMode("GUIDED"):
        log("Can't switch to GUIDED, aborting mission")
        return
    
    # attempt to arm
    try:
        vehicle.arm(True, 5)
    except:
        log("ARM FAILED, aborting mission")
        return

    log("ARMED SUCCESS!")

    time.sleep(5)

    # official mission start
    started_mission = True
    c = datetime.now()
    mission_start_time = c.strftime('%H:%M:%S')

    total_items = len(cmd_items)
    previous_command = -1
    waypoint_start_time = time.time()
    item_start_time = mission_start_time
    current_mission_item = None
    target_alt = 4
    descending = False
    has_payload = False
    payload_expected = False
    item_count = 1
    pos_buff = []

    while in_autonomous_mode() and len(cmd_items) > 0:
        if vehicle.commands.next > previous_command or vehicle.commands.next == len(vehicle.commands):
            #starting new waypoint
            vehicle.mode = VehicleMode("GUIDED")

            item_count += 1
            current_mission_item = cmd_items.pop(0)
            while current_mission_item.command != CommandType.WAYPOINT and current_mission_item.command != CommandType.SEARCH:
                log("starting command " + str(current_mission_item.command))
                item_start_time = datetime.now().strftime('%H:%M:%S')
                send_mission_status(mission_title, mission_start_time, item_start_time, total_items, current_mission_item.title, get_item_title(cmd_items, 0), get_item_title(cmd_items, 1))

                if current_mission_item.command == CommandType.TAKEOFF:

                    vehicle.commands.clear()
                    print ("uploading: " + str(len(cmd_items)))
                    for item in cmd_items:
                        item.add_commands(vehicle)
                    time.sleep(1)
                    vehicle.commands.upload()

                    result = offboard_utils.takeoff(vehicle, current_mission_item.target)
                    if not result:
                        log("Takeoff failed, aborting mission")
                        started_mission = False
                        return
                    
                if current_mission_item.command == CommandType.PICKUP_PAYLOAD:
                    if not payload_expected:
                        log("Object wasn't found, aborting mission")
                        started_mission = False
                        vehicle.mode = VehicleMode("LAND")
                        return

                    attempts = 0
                    while not has_payload and attempts < current_mission_item.target and in_autonomous_mode():
                        log("Checking payload, will takeoff to 3m")
                        result = offboard_utils.takeoff(vehicle, 3)
                        if not result:
                            log("Takeoff failed, aborting mission")
                            started_mission = False
                            return
                        
                        has_payload = check_for_payload()
                        if not has_payload:
                            log("Payload not detected")
                            for i in range(5):
                                if check_for_bag(down_frame) is not None:
                                    success = land_on_object(rate)
                                    break
                        else:
                            log("Payload Verified")

                        attempts += 1
                    
                if current_mission_item.command == CommandType.GRIPPER:
                    offboard_utils.set_gripper(vehicle, current_mission_item.target)

                if current_mission_item.command == CommandType.FINISH:
                    vehicle.mode = VehicleMode("LAND")
                    log("Mission Completed")
                    started_mission = False
                    return

                item_count += 1
                if len(cmd_items) > 0:
                    current_mission_item = cmd_items.pop(0)

            item_start_time = datetime.now().strftime('%H:%M:%S')
            send_mission_status(mission_title, mission_start_time, item_start_time, total_items, current_mission_item.title, get_item_title(cmd_items, 0), get_item_title(cmd_items, 1))

            print("starting waypoint: " + str(vehicle.commands.next))
            missionitem = vehicle.commands[vehicle.commands.next]
            if missionitem.x == 0 or missionitem.y == 0:
                missionitem = vehicle.commands[vehicle.commands.next + 1]

            #turn to heading before continuing
            lat = missionitem.x
            lon = missionitem.y
            alt = missionitem.z
            target = LocationGlobalRelative(lat, lon, alt)
            target_alt = alt
            descending = alt < vehicle.location.global_relative_frame.alt

            previous_command = vehicle.commands.next

            if lat != 0 or lon != 0:
                heading = offboard_utils.get_heading(vehicle, target)
                offboard_utils.turn_towards(vehicle, heading, alt)
                print("reached heading")
                
                waypoint_start_time = time.time()
                vehicle.mode = VehicleMode("AUTO")
                vehicle.airspeed = current_mission_item.speed
                previous_command = vehicle.commands.next + 1

        else:
            if check_obstacles(1) and in_autonomous_mode():
                #failsafe obstacle avoidance
                log("Too close to obstacle!")
                vehicle.mode = VehicleMode("GUIDED")
                offboard_utils.send_velocity_for(vehicle, -0.5, 0, 0, 0.5)
                vehicle.mode = VehicleMode("AUTO")

            elif descending and check_downward_obs() != -1 and in_autonomous_mode():
                #downward obstacle avoidance
                t = datetime.now().strftime('%H:%M:%S')
                send_mission_status(mission_title, mission_start_time, t, total_items, current_mission_item.title, "Downward Obstacle Detected", get_item_title(cmd_items, 0))
                vehicle.mode = VehicleMode("GUIDED")
                down_obs = check_downward_obs()
                while down_obs != -1 and in_autonomous_mode():
                    if down_obs == 0:
                        offboard_utils.send_frame_velocity(vehicle, 0.5, 0, 0, 0)
                    elif down_obs == 1:
                        offboard_utils.send_frame_velocity(vehicle, 0, 0.5, 0, 0)
                    elif down_obs == 2:
                        offboard_utils.send_frame_velocity(vehicle, -0.5, 0, 0, 0)
                    else:
                        offboard_utils.send_frame_velocity(vehicle, 0, -0.5, 0, 0)
                    time.sleep(0.5)
                    down_obs = check_downward_obs()
                    send_mission_status(mission_title, mission_start_time, item_start_time, total_items, current_mission_item.title, get_item_title(cmd_items, 0),  get_item_title(cmd_items, 1))
                vehicle.mode = VehicleMode("AUTO")
                
            elif (vehicle.rangefinder.distance < target_alt - 2 or vehicle.rangefinder.distance < 1) and VehicleMode("AUTO") and in_autonomous_mode() \
                  and vehicle.location.global_relative_frame.alt < 5 and not (current_mission_item.object_search and check_for_bag(down_frame) is not None) and vehicle.armed:
                #failsafe low altitude
                log("Correcting Altitude")
                yaw = vehicle.attitude.yaw
                vehicle.mode = VehicleMode("GUIDED")
                while vehicle.rangefinder.distance < target_alt - 2 or (vehicle.rangefinder.distance < 1 and in_autonomous_mode() and vehicle.armed):
                    offboard_utils.send_frame_velocity(vehicle, 0, 0, -0.5, yaw)
                    time.sleep(0.5)
                vehicle.mode = VehicleMode("AUTO")

            elif current_mission_item != None and time.time() - waypoint_start_time > current_mission_item.timeout and current_mission_item.timeout != -1:
                # waypoint probably unreachable
                log("Waypoint timed out")
                vehicle.commands.next = previous_command + 1

            elif current_mission_item.land_search and check_safe_landing() and in_autonomous_mode():
                #landing zone found
                t = datetime.now().strftime('%H:%M:%S')
                send_mission_status(mission_title, mission_start_time, t, total_items, current_mission_item.title, "Landing... ", get_item_title(cmd_items, 0))
                vehicle.mode = VehicleMode("LAND")

            elif current_mission_item.object_search and check_for_bag(down_frame) is not None and in_autonomous_mode():

                #object detected
                t = datetime.now().strftime('%H:%M:%S')
                send_mission_status(mission_title, mission_start_time, t, total_items, current_mission_item.title, "Landing on Object", get_item_title(cmd_items, 0))
                vehicle.mode = VehicleMode("GUIDED")
                success = land_on_object(rate)

                if not success:
                    # return to searching
                    if in_autonomous_mode():
                        vehicle.mode = VehicleMode("AUTO")
                else:
                    # drone successfully landed
                    payload_expected = True

                    skipped = 0
                    while len(cmd_items) > 0 and cmd_items[0].command == CommandType.SEARCH:
                        cmd_items.pop(0)
                        skipped += 2
                    vehicle.commands.next = previous_command + skipped

                item_start_time = datetime.now().strftime('%H:%M:%S')
                

        rate.sleep()

    log("Mission Completed")
    if in_autonomous_mode():
        vehicle.mode = VehicleMode("GUIDED")

    started_mission = False


def log(message):
    print(message)
    msg_pub.publish(message)

offboard_utils.log = log

def in_autonomous_mode():
    return (vehicle.mode == VehicleMode("GUIDED") or vehicle.mode == VehicleMode("AUTO")) and not rospy.is_shutdown()


def camera_callback (img):
    global down_frame
    down_frame = bridge.imgmsg_to_cv2 (img)
    global last_frame_update
    last_frame_update = time.time()

def down_depth_callback(img):
    global down_depth
    down_depth = bridge.imgmsg_to_cv2(img)

def front_depth_callback(img):
    global front_depth
    front_depth = bridge.imgmsg_to_cv2(img)

# Callback function for vehicle's 'armed' attribute
def arm_callback(self, attr_name, value):
    state_pub.publish(value)
    print ("callback " + str(value))

def recieve_camera_info(msg):
    global cameraInfo
    cameraInfo = msg   

def command_callback(command):
    command_str = command.data
    args = command_str.split()
    if args[0] == "Pickup":
        bag_mission((float(args[1]), float(args[2])))
    elif args[0] == "Search":
        bag_land_mission()
    elif args[0] == "Summon":
        summon_mission((float(args[1]), float(args[2])))
    elif args[0] == "Gripper":
        offboard_utils.set_gripper(vehicle, not offboard_utils.get_gripper_status())
    elif args[0] == "SafeLand":
        test_vel_mission()
    elif args[0] == "Reboot":
        vehicle.reboot()
    else:
        log("Invalid command: " + command_str)

    #TODO: safe land, safe RTL
    
def start_node():

    global bridge
    bridge = CvBridge()

    rospy.Subscriber('/down/color/image_raw', Image, camera_callback, queue_size=1)
    rospy.Subscriber('/down/depth/image_rect_raw', Image, down_depth_callback, queue_size=1)
    rospy.Subscriber('/front/color/camera_info', CameraInfo, recieve_camera_info)
    rospy.Subscriber('/front/depth/image_rect_raw', Image, front_depth_callback, queue_size=1)
    rospy.Subscriber('target_coordinates', Point, bag_mission)
    rospy.Subscriber("commands", String, command_callback)
    global state_pub
    state_pub = rospy.Publisher('is_armed', Bool, queue_size=1)
    global msg_pub
    msg_pub = rospy.Publisher('messages', String, queue_size=5)

    global rate
    rate = rospy.Rate(20)

    print ("waiting for rospy...")

    # Wait for rospy
    while rospy.is_shutdown():
        rate.sleep()

    print ("connecting to vehicle...")

    global vehicle
    vehicle = connect('127.0.0.1:14540', baud=921600, wait_ready=True)

    # Add arm_callback as a listener for 'armed' attribute changes
    vehicle.add_attribute_listener('armed', arm_callback)

    log("offboard node connected")

    offboard_utils.set_gripper(vehicle, True)

    #mission started with tcp message
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("offboard_node")
    rospy.loginfo("offbard node started")
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass 
