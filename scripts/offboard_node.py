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
from offboard_utils import rangefinder_distance
from mission_utils import MissionItem, CommandType, add_search_waypoints
from std_msgs.msg import String
from datetime import datetime
import mission_definitions

vehicle = None
front_depth = None
down_frame = None
down_depth = None
cameraInfo = None
last_frame_update = 0

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

def check_for_bag(frame):
    if vehicle is None or frame is None:
        print("no frame")
        return None
    if time.time() - last_frame_update > 1:
        print("time since last frame " + str(time.time() - last_frame_update))
        return None

    alt = vehicle.location.global_relative_frame.alt
    pixel_size = offboard_utils.world_to_pixel((0.5, 0.5, alt), cameraInfo) # max pixel size of bag
    detection = None
    if rangefinder_distance(vehicle) < 0.8:
        detection = detectWhite(frame, pixel_size[0] * pixel_size[1])
    else:
        detection = find_bag(frame)
    return detection

# (horizontal, vertical)
def get_bag_image_coords():
    detection = check_for_bag(down_frame)
    if detection is None:
        return None
    pixel_offsets = offboard_utils.get_offsets(vehicle, down_frame, cameraInfo)
    x0, x1, y0, y1 = detection
    width = x1 - x0
    height = y1 - y0
    centerx = (x0 + x1) // 2 - (down_frame.shape[1] / 2)
    centery = (y0 + y1) // 2 - (down_frame.shape[0] / 2)
    if centerx > 0:
        centered_x = x0 + (width // 4) - (down_frame.shape[1] / 2)
    else:
        centered_x = x0 + (width // 4 * 3) - (down_frame.shape[1] / 2)
    if centery > 0:
        centered_y = y0 + (height // 4) - (down_frame.shape[0] / 2)
    else:
        centered_y = y0 + (height // 4 * 3) - (down_frame.shape[0] / 2)
    coordinates = ((centered_x + pixel_offsets[0]) / down_frame.shape[1],
                   (centered_y + pixel_offsets[1]) / -down_frame.shape[0])
    pixel_x = centerx + pixel_offsets[0]
    pixel_y = centery + pixel_offsets[1]
    return (coordinates, (pixel_x, pixel_y))

def search_for_bag(rate):
    vehicle.simple_goto(vehicle.location.global_frame)
    while not rospy.is_shutdown() and vehicle.armed:
        if check_for_bag(down_frame) is not None:
            land_on_object(rate)
        rate.sleep()

def estimate_velocity(obj_pos_history):
    if len(obj_pos_history) <= 5:
        return
    pos1 = obj_pos_history[-5]
    pos2 = obj_pos_history[-1]  # skip one frame
    if math.degrees(abs(pos1[3] - pos2[3])) > 5:
        return
    time_diff = pos2[4] - pos1[4]
    velocity_vector = ((pos2[0] - pos1[0]) / time_diff,
                       (pos2[1] - pos1[1]) / time_diff,
                       (pos2[2] - pos1[2]) / time_diff)
    print("Estimated: " + str((velocity_vector[1], velocity_vector[0], velocity_vector[2])))
    print("True: " + str(vehicle.velocity))
    msg = vehicle.message_factory.vision_speed_estimate_encode(
        velocity_vector[1], velocity_vector[0], velocity_vector[2],
        [float("NAN"), 0, 0, 0, 0, 0, 0, 0, 0]
    )
    vehicle.send_mavlink(msg)
    return velocity_vector

def check_obstacles(min_dist):
    if vehicle.location.global_relative_frame.alt < 1:
        return False
    for c in range(100, front_depth.shape[1] - 100, 20):
        count = 0
        for r in range(100, front_depth.shape[0] - 100, 10):
            dist_m = front_depth[r, c] * 0.001
            if dist_m > 0.2 and dist_m < min_dist:
                count += 1
        if count > 3:
            return True
    return False

def check_downward_obs(min_dis=3):
    if vehicle.location.global_relative_frame.alt < min_dis + 1.5:
        return -1
    top = [0, 0]
    right = [0, 0]
    bottom = [0, 0]
    left = [0, 0]
    obstacle = False
    for r in range(0, down_depth.shape[0])[10:-10:20]:
        count = 0
        for c in range(0, down_depth.shape[1])[10:-10:10]:
            depth = down_depth[r][c] * 0.001
            if depth < min_dis and depth > 0.8 and depth != 0:
                count += 1
            if depth > 0.8 and depth != 0:
                if r < down_depth.shape[0] * 0.33:
                    top[0] += depth
                    top[1] += 1
                if c > down_depth.shape[1] * 0.66:
                    right[0] += depth
                    right[1] += 1
                if r > down_depth.shape[0] * 0.66:
                    bottom[0] += depth
                    bottom[1] += 1
                if c < down_depth.shape[1] * 0.33:
                    left[0] += depth
                    left[1] += 1
        if count > 3:
            obstacle = True
    if not obstacle:
        return -1
    topD = top[0] / top[1]
    rightD = right[0] / right[1]
    bottomD = bottom[0] / bottom[1]
    leftD = left[0] / left[1]
    directions = [topD, rightD, bottomD, leftD]
    best_dir = np.argmax(directions)
    return best_dir

def check_safe_landing(num_bins=5, bin_size=50):
    bins = [0] * (num_bins * num_bins)
    points = [0] * (num_bins * num_bins)
    startr = int(down_depth.shape[0] / 2 - (num_bins / 2.0) * bin_size)
    startc = int(down_depth.shape[1] / 2 - (num_bins / 2.0) * bin_size)
    for r in range(startr, down_depth.shape[0] - startr)[::10]:
        for c in range(startc, down_depth.shape[1] - startc)[::10]:
            binr = int((r - startr) / bin_size)
            binc = int((c - startc) / bin_size)
            bin = binr * num_bins + binc
            if down_depth[r][c] * 0.001 > 0.2:
                bins[bin] += down_depth[r][c] * 0.001
                points[bin] += 1
    for b in range(len(bins)):
        bins[b] = bins[b] / points[b]
    avg_depth = sum(bins) / len(bins)
    print(bins)
    avg_deviation = sum(abs(d - avg_depth) for d in bins) / len(bins)
    print(("SAFE: " if avg_deviation < 0.2 else "UNSAFE: ") + str(avg_deviation))
    return avg_deviation < 0.2

def try_land(rate):
    yaw = vehicle.attitude.yaw
    while check_safe_landing():
        print("trying safe landing")
        alt = rangefinder_distance(vehicle)
        offboard_utils.send_frame_velocity(vehicle, 0, 0, 1, yaw)
        if alt < 1.5:
            vehicle.mode = VehicleMode("LAND")
            return True
        rate.sleep()
    return False

def check_for_payload():
    yaw = vehicle.attitude.yaw
    for i in range(10):
        if i < 5:
            offboard_utils.send_frame_velocity(vehicle, 1, 0, 0, yaw)
        else:
            offboard_utils.send_frame_velocity(vehicle, -1, 0, 0, yaw)
        close_points = 0
        for r in range(0, down_depth.shape[0], 10):
            for c in range(0, down_depth.shape[1], 10):
                depth = down_depth[r][c] * 0.001
                if depth < 0.75 and depth != 0:
                    close_points += 1
        print(close_points)
        if close_points > 5:
            offboard_utils.send_frame_velocity(vehicle, 0, 0, 0, yaw)
            return True
        time.sleep(0.2)
    offboard_utils.send_frame_velocity(vehicle, 0, 0, 0, yaw)
    return False

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
    start_alt = rangefinder_distance(vehicle)
    relative_coordinates = None
    print("landing on object")
    tolerance = 0.25
    def within_tolerance(multiplier=1):
        return abs(relative_coordinates[0]) < tolerance * multiplier and abs(relative_coordinates[1]) < tolerance * multiplier
    pos_buff = []
    final_descent = False
    final_descent_height = 0.6
    centered_frames = 0
    centered = False
    lost_for = 0
    while not final_descent and in_autonomous_mode():
        alt = rangefinder_distance(vehicle)
        object_coordinates = get_bag_image_coords()
        if object_coordinates is None:
            print("target lost")
            lost_for += 1
            if lost_for > 2:
                offboard_utils.send_frame_velocity(vehicle, 0, 0, -0.3, yaw)
                centered_frames = 0
            if alt > start_alt + 0.5 or lost_for > 10:
                break
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
        if (within_tolerance() and alt < final_descent_height and centered_frames >= 3) or alt < 0.4:
            final_descent = True
            print("final descent")
            break
        adjustment_speed = 1 if alt > 1.5 else 0.75
        if within_tolerance(0.5):
            adjustment_speed = 0
        max = 0.5
        descent_speed = 0.3 if alt > 2 else 0.2
        if alt < final_descent_height:
            descent_speed = 0
        if alt < final_descent_height * 0.8:
            descent_speed = -0.2
        x_speed = clamp(relative_coordinates[0] * adjustment_speed, max)
        y_speed = clamp(relative_coordinates[1] * adjustment_speed, max)
        if not within_tolerance():
            centered_frames = 0
            centered = False
            offboard_utils.send_frame_velocity(vehicle, y_speed, x_speed, -0.2 if alt < 1 else 0, yaw)
        else:
            centered_frames += 1
            offboard_utils.send_frame_velocity(vehicle, y_speed, x_speed, descent_speed, yaw)
        rate.sleep()
    if not final_descent:
        print("landing attempt failed")
        vehicle.airspeed = 1
        vehicle.simple_goto(start_pos)
        return False
    offboard_utils.set_gripper(vehicle, False)
    vehicle.mode = VehicleMode("LAND")
    time.sleep(5)
    offboard_utils.set_gripper(vehicle, True)
    time.sleep(2)
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)
    return True

started_mission = False

def pre_arm_checks():
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    if not vehicle.is_armable or not vehicle.mode == VehicleMode("GUIDED"):
        log("Can't switch to GUIDED, aborting mission")
        return False
    try:
        vehicle.arm(True, 5)
    except:
        log("ARM FAILED, aborting mission")
        return False
    log("ARMED SUCCESS!")
    time.sleep(5)
    return True

from mission_context import MissionContext

def handle_non_waypoint_commands(context: MissionContext, current_mission_item):
    global started_mission
    log("starting command " + str(current_mission_item.title))
    context.item_start_time = datetime.now().strftime('%H:%M:%S')
    send_mission_status(context.mission_title, context.mission_start_time, context.item_start_time, context.total_items,
                        current_mission_item.title, get_item_title(context.cmd_items, 0), get_item_title(context.cmd_items, 1))
    if current_mission_item.command == CommandType.TAKEOFF:
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.commands.clear()
        print("uploading: " + str(len(context.cmd_items)))
        for item in context.cmd_items:
            item.add_commands(vehicle)
        time.sleep(1)
        vehicle.commands.upload()
        result = offboard_utils.takeoff(vehicle, current_mission_item.target)
        if not result:
            log("Takeoff failed, aborting mission")
            started_mission = False
            return False
        return True
    if current_mission_item.command == CommandType.PICKUP_PAYLOAD:
        vehicle.mode = VehicleMode("GUIDED")
        if not context.payload_expected:
            log("Object wasn't found, aborting mission")
            started_mission = False
            vehicle.mode = VehicleMode("LAND")
            return False
        attempts = 0
        while not context.has_payload and in_autonomous_mode():
            log("Checking payload, will takeoff to 3m")
            result = offboard_utils.takeoff(vehicle, 3)
            if not result:
                log("Takeoff failed, aborting mission")
                started_mission = False
                return False
            context.has_payload = check_for_payload()
            if not context.has_payload:
                log("Payload not detected")
                if attempts > current_mission_item.target:
                    log("giving up payload")
                    break
                for i in range(5):
                    if check_for_bag(down_frame) is not None:
                        success = land_on_object(context.rate)
                        break
            else:
                log("Payload Verified")
            attempts += 1
        return True
    if current_mission_item.command == CommandType.GRIPPER:
        offboard_utils.set_gripper(vehicle, current_mission_item.target)
        return True
    if current_mission_item.command == CommandType.DELAY:
        time.sleep(current_mission_item.target)
        return True
    if current_mission_item.command == CommandType.FINISH:
        vehicle.mode = VehicleMode("LAND")
        log("Mission Finished Successfully")
        started_mission = False
        return False
    return True

def handle_obstacle_and_altitude(context: MissionContext, current_mission_item):
    # This helper function is extracted from do_mission.
    # It does not modify external variables directly; its side effects are performed on the vehicle.
    if check_obstacles(1) and in_autonomous_mode():
        log("Too close to obstacle!")
        vehicle.mode = VehicleMode("GUIDED")
        offboard_utils.send_velocity_for(vehicle, -0.5, 0, 0, 1)
        offboard_utils.send_velocity_for(vehicle, 0, 0, 0, 1)
        if in_autonomous_mode():
            vehicle.mode = VehicleMode("AUTO")

    elif context.descending and check_downward_obs() != -1 and in_autonomous_mode() and (not context.has_payload):
        t = datetime.now().strftime('%H:%M:%S')
        send_mission_status(context.mission_title, context.mission_start_time, t, context.total_items,
                            current_mission_item.title, "Downward Obstacle Detected", get_item_title(context.cmd_items, 0))
        vehicle.mode = VehicleMode("GUIDED")
        down_obs = check_downward_obs()
        while down_obs != -1 and in_autonomous_mode():
            log("Downward Obstacle Detected")
            if down_obs == 0:
                offboard_utils.send_velocity_for(vehicle, 0.5, 0, 0, 1)
            elif down_obs == 1:
                offboard_utils.send_velocity_for(vehicle, 0, 0.5, 0, 1)
            elif down_obs == 2:
                offboard_utils.send_velocity_for(vehicle, -0.5, 0, 0, 1)
            else:
                offboard_utils.send_velocity_for(vehicle, 0, -0.5, 0, 1)
            offboard_utils.send_velocity_for(vehicle, 0, 0, 0, 1)
            down_obs = check_downward_obs()
        send_mission_status(context.mission_title, context.mission_start_time, context.item_start_time, context.total_items,
                            current_mission_item.title, get_item_title(context.cmd_items, 0), get_item_title(context.cmd_items, 1))
        if in_autonomous_mode():
            vehicle.mode = VehicleMode("AUTO")

    elif (rangefinder_distance(vehicle) < context.target_alt - 2 or rangefinder_distance(vehicle) < 1) and in_autonomous_mode() and rangefinder_distance(vehicle) > 0 \
         and vehicle.location.global_relative_frame.alt < 5 and not (current_mission_item.object_search and check_for_bag(down_frame) is not None) \
         and vehicle.armed and (not context.has_payload):
        log("Correcting Low Altitude")
        yaw = vehicle.attitude.yaw
        vehicle.mode = VehicleMode("GUIDED")
        while rangefinder_distance(vehicle) < context.target_alt - 2 or (rangefinder_distance(vehicle) < 1 and in_autonomous_mode() and vehicle.armed):
            offboard_utils.send_frame_velocity(vehicle, 0, 0, -0.5, yaw)
            time.sleep(0.5)
        vehicle.mode = VehicleMode("AUTO")

    elif in_autonomous_mode() and context.target_alt != 0 and vehicle.location.global_relative_frame.alt > context.target_alt + 1 and (not context.descending):
        log("Correcting High Altitude")
        yaw = vehicle.attitude.yaw
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.location.global_relative_frame.alt > context.target_alt and in_autonomous_mode() and vehicle.armed:
            offboard_utils.send_frame_velocity(vehicle, 0, 0, 0.5, yaw)
            time.sleep(0.5)
        vehicle.mode = VehicleMode("AUTO")

    elif current_mission_item is not None and time.time() - context.waypoint_start_time > current_mission_item.timeout and current_mission_item.timeout != -1:
        log("Waypoint timed out")
        vehicle.commands.next = context.previous_command + 1

    elif current_mission_item.land_search and vehicle.location.global_relative_frame.alt < current_mission_item.target.alt + 1 and check_safe_landing() and in_autonomous_mode():
        t = datetime.now().strftime('%H:%M:%S')
        send_mission_status(context.mission_title, context.mission_start_time, t, context.total_items,
                            current_mission_item.title, "Landing... ", get_item_title(context.cmd_items, 0))
        vehicle.mode = VehicleMode("LAND")

    elif current_mission_item.object_search and vehicle.location.global_relative_frame.alt < current_mission_item.target.alt + 1 and \
         check_for_bag(down_frame) is not None and in_autonomous_mode():
        t = datetime.now().strftime('%H:%M:%S')
        send_mission_status(context.mission_title, context.mission_start_time, t, context.total_items,
                            current_mission_item.title, "Landing on Object", get_item_title(context.cmd_items, 0))
        vehicle.mode = VehicleMode("GUIDED")
        success = land_on_object(context.rate)
        if not success:
            if in_autonomous_mode():
                vehicle.mode = VehicleMode("AUTO")
        else:
            # Note: payload_expected might be updated outside; here we simply log.
            log("Payload acquired after landing on object")
            skipped = 0
            while len(context.cmd_items) > 0 and context.cmd_items[0].command == CommandType.SEARCH:
                context.cmd_items.pop(0)
                skipped += 2
            vehicle.commands.next = context.previous_command + skipped
        context.item_start_time = datetime.now().strftime('%H:%M:%S')

from mission_context import MissionContext

def do_mission(mission_title, cmd_items):
    global started_mission
    # Initialize mission context
    context = MissionContext(launch_pos=vehicle.location.global_relative_frame, mission_title=mission_title, rate=rate)
    context.cmd_items = cmd_items.copy()
    context.total_items = len(cmd_items)
    context.previous_command = -1
    context.waypoint_start_time = time.time()
    context.item_start_time = ""
    previous_goal = (0, 0)
    item_count = 1
    pos_buff = []

    # Upload commands
    log("uploading mission...")
    vehicle.commands.clear()
    print("Number of cmd_items: " + str(len(cmd_items)))
    for item in cmd_items:
        item.add_commands(vehicle)
    time.sleep(1)
    vehicle.commands.upload()
    # Pre-arm checks
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    if not vehicle.is_armable or not vehicle.mode == VehicleMode("GUIDED"):
        log("Can't switch to GUIDED, aborting mission")
        return
    try:
        vehicle.arm(True, 5)
    except:
        log("ARM FAILED, aborting mission")
        return
    log("ARMED SUCCESS!")
    time.sleep(5)
    started_mission = True
    c = datetime.now()
    context.mission_start_time = c.strftime('%H:%M:%S')

    while in_autonomous_mode() and len(context.cmd_items) > 0:
        if vehicle.commands.next > context.previous_command or vehicle.commands.next == len(vehicle.commands):
            item_count += 1
            current_mission_item = context.cmd_items.pop(0)
            # Process non-waypoint commands
            while current_mission_item.command != CommandType.WAYPOINT and current_mission_item.command != CommandType.SEARCH:
                result = handle_non_waypoint_commands(context, current_mission_item)
                if not result:
                    return
                item_count += 1
                if len(context.cmd_items) > 0:
                    current_mission_item = context.cmd_items.pop(0)
            if vehicle.commands.next == 0:
                print("Waiting for WP 1")
                vehicle.mode = VehicleMode("AUTO")
                continue
            print(current_mission_item.title)
            context.item_start_time = datetime.now().strftime('%H:%M:%S')
            send_mission_status(context.mission_title, context.mission_start_time, context.item_start_time, context.total_items, current_mission_item.title, get_item_title(context.cmd_items, 0), get_item_title(context.cmd_items, 1))
            missionitem = vehicle.commands[vehicle.commands.next - 1]
            print("starting waypoint: " + str(vehicle.commands.next))
            print(str(missionitem.x) + " " + str(missionitem.y))
            lat = missionitem.x
            lon = missionitem.y
            alt = missionitem.z
            target = LocationGlobalRelative(lat, lon, alt)
            context.target_alt = alt
            context.descending = alt < vehicle.location.global_relative_frame.alt and alt != 0
            context.previous_command = vehicle.commands.next
            if lat != 0 or lon != 0:
                print(previous_goal)
                print((lat, lon))
                if (previous_goal[0] != lat or previous_goal[1] != lon) and (previous_goal[0] != 0 or previous_goal[1] != 0):
                    vehicle.mode = VehicleMode("GUIDED")
                    log("turning to heading")
                    heading = offboard_utils.get_heading(vehicle, target)
                    offboard_utils.turn_towards(vehicle, heading, alt)
                    log("reached heading")
                    time.sleep(4)
                context.waypoint_start_time = time.time()
                vehicle.airspeed = current_mission_item.speed
                context.previous_command = vehicle.commands.next + 1
                previous_goal = (lat, lon)
            vehicle.mode = VehicleMode("AUTO")
        else:
            handle_obstacle_and_altitude(context, current_mission_item)
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

def camera_callback(img):
    global down_frame
    down_frame = bridge.imgmsg_to_cv2(img)
    global last_frame_update
    last_frame_update = time.time()

def down_depth_callback(img):
    global down_depth
    down_depth = bridge.imgmsg_to_cv2(img)

def front_depth_callback(img):
    global front_depth
    front_depth = bridge.imgmsg_to_cv2(img)

def arm_callback(self, attr_name, value):
    state_pub.publish(value)
    print("callback " + str(value))

def recieve_camera_info(msg):
    global cameraInfo
    cameraInfo = msg

def command_callback(command):
    command_str = command.data
    args = command_str.split()
    if args[0] == "Pickup":
        launch_pos = vehicle.location.global_relative_frame
        cmds = mission_definitions.bag_mission((float(args[1]), float(args[2])), launch_pos)
        do_mission("Pickup Poop", cmds)
    elif args[0] == "Search":
        launch_pos = vehicle.location.global_relative_frame
        cmds = mission_definitions.bag_land_mission(launch_pos)
        do_mission("Search for Bag", cmds)
    elif args[0] == "Summon":
        launch_pos = vehicle.location.global_relative_frame
        cmds = mission_definitions.summon_mission((float(args[1]), float(args[2])), launch_pos)
        do_mission("Go to Target", cmds)
    elif args[0] == "Gripper":
        offboard_utils.set_gripper(vehicle, not offboard_utils.get_gripper_status())
    elif args[0] == "SafeLand":
        launch_pos = vehicle.location.global_relative_frame
        cmds = mission_definitions.start_safe_land(launch_pos)
        do_mission("Safe Land", cmds)
    elif args[0] == "Reboot":
        vehicle.reboot()
    else:
        log("Invalid command: " + command_str)


def start_node():
    global bridge
    bridge = CvBridge()
    rospy.Subscriber('/down/color/image_raw', Image, camera_callback, queue_size=1)
    rospy.Subscriber('/down/depth/image_rect_raw', Image, down_depth_callback, queue_size=1)
    rospy.Subscriber('/front/color/camera_info', CameraInfo, recieve_camera_info)
    rospy.Subscriber('/front/depth/image_rect_raw', Image, front_depth_callback, queue_size=1)
    rospy.Subscriber("commands", String, command_callback)
    global state_pub
    state_pub = rospy.Publisher('is_armed', Bool, queue_size=1)
    global msg_pub
    msg_pub = rospy.Publisher('messages', String, queue_size=5)
    global rate
    rate = rospy.Rate(20)
    print("waiting for rospy...")
    while rospy.is_shutdown():
        rate.sleep()
    print("connecting to vehicle...")
    global vehicle
    vehicle = connect('127.0.0.1:14540', baud=921600, wait_ready=True)
    vehicle.add_attribute_listener('armed', arm_callback)
    log("offboard node connected")
    offboard_utils.set_gripper(vehicle, True)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("offboard_node")
    rospy.loginfo("offbard node started")
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
