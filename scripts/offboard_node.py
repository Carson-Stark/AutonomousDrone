#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from BlobDetection import BlobDetection
from cv_bridge import CvBridge
import cv2
import numpy as np
from dronekit import connect, VehicleMode, mavutil, LocationGlobalRelative, Command
import time
import math
from drone.srv import Coordinates
import pyrealsense2 as rs2

vehicle = None

def takeoff(targetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ("Taking off!")
    vehicle.simple_takeoff(targetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height
    while vehicle.armed:
        print (" Altitude: " + str(vehicle.location.global_relative_frame.alt))
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=targetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(0.5)

# velocity_x > 0 => fly foward
# velocity_x < 0 => fly backward
# velocity_y > 0 => fly right
# velocity_y < 0 => fly left
# velocity_z < 0 => ascend
# velocity_z > 0 => descend
def send_frame_velocity(velocity_x, velocity_y, velocity_z, yaw):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)
    set_yaw(yaw)

def brake_for(seconds, yaw):
    sleep_increment = 0.5
    count = 0
    while count < seconds:
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            0, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        vehicle.send_mavlink(msg)
        set_yaw(yaw)
        time.sleep(sleep_increment)
        count += sleep_increment

def set_yaw(heading):
    """
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    """
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    
    # send command to vehicle
    vehicle.send_mavlink(msg)

def set_gripper(closed):
	msg = vehicle.message_factory.command_long_encode(
		0, 0, 
		mavutil.mavlink.MAV_CMD_DO_GRIPPER,
		0,
		1, #servo number
		1 if closed else 0, #open/closes
		0,0,0,0,0
		)
	vehicle.send_mavlink(msg)

def distance_meters(target):
    return (abs(vehicle.location.global_relative_frame.lat - target.lat) * 111139, abs(vehicle.location.global_relative_frame.lon - target.lon) * 111139) 

def fly_to_target(rate, target, timeout, tolerance=0.5):
    vehicle.simple_goto(vehicle.location.global_relative_frame)

    cmds = vehicle.commands
    cmds.clear()
    cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, target.lat, target.lon, target.alt)
    cmds.add(cmd1)
    vehicle.airspeed = 1
    print("uploading...")
    cmds.upload() # Send commands
    while vehicle.mode != VehicleMode("AUTO"):
        rate.sleep()
        vehicle.mode = VehicleMode("AUTO")
        print("trying to switch to auto")

    start_time = time.time()
    dist = distance_meters(target)
    while vehicle.mode == VehicleMode("GUIDED") and time.time() - start_time < timeout and dist[0] > tolerance or dist[1] > tolerance:
        print(str(dist))
        rate.sleep()

    print("reached target")
    vehicle.mode = VehicleMode("GUIDED")
        

def check_for_bag (frame):
    #detections = object_detector.detect(frame)
    alt = vehicle.location.global_relative_frame.alt
    pixel_size = world_to_pixel((0.5, 0.5, alt)) #max pixel size of bag
    detection = object_detector.detectWhite(frame, pixel_size[0] * pixel_size[1])
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
    
    pixel_offsets = get_offsets()
    if detection[2][0] + int(detection[2][2]/2) > down_frame.shape[1] / 2:
        centered_pixel_x = float(detection[2][0] + int(detection[2][2]/4)) - (down_frame.shape[1] / 2)
    else:
        centered_pixel_x = float(detection[2][0] + int(detection[2][2] * 0.75)) - (down_frame.shape[1] / 2)
    if detection[2][1] + int(detection[2][3]/2) > down_frame.shape[0] / 2:
        centered_pixel_y = float(detection[2][1] + int(detection[2][3]/4)) - (down_frame.shape[0] / 2)
    else:
        centered_pixel_y = float(detection[2][1] + int(detection[2][3] * 0.75)) - (down_frame.shape[0] / 2)
    coordinates = ((centered_pixel_x + pixel_offsets[0]) / down_frame.shape[1], \
                   (centered_pixel_y + pixel_offsets[1]) / -down_frame.shape[0])
    pixel_x = float(detection[2][0] + int(detection[2][2]/2)) + pixel_offsets[0]
    pixel_y = float(detection[2][1] + int(detection[2][3]/2)) + pixel_offsets[1]
    return (coordinates, (centered_pixel_x + (down_frame.shape[1] / 2), centered_pixel_y + (down_frame.shape[0] / 2)))

def search_for_bag (rate):
    vehicle.simple_goto(vehicle.location.global_frame)
    while not rospy.is_shutdown() and vehicle.armed:
        if check_for_bag(down_frame) is not None:
            land_on_object(rate)
        rate.sleep()

def sign(num):
    return 1 if num > 0 else -1

#account for roll and pitch in image coordinates
def get_offsets():
    roll = vehicle.attitude.roll
    pitch = vehicle.attitude.pitch

    alt = vehicle.location.global_relative_frame.alt
    if alt > 4:
        return (0, 0)
    
    # Calculate offsets using trigonometry
    horizontal_offset = alt * math.tan(roll)
    vertical_offset = alt * math.tan(pitch)

    pixel_coord = world_to_pixel((horizontal_offset, vertical_offset, alt))
    return (-(pixel_coord[0] - (down_frame.shape[1] / 2)), -(pixel_coord[1] - (down_frame.shape[0] / 2)))

#(X = right, Y = foward)
def rotate_to_global (local_target):
    yaw = -vehicle.attitude.yaw

    x = math.cos(yaw) * local_target[0] + math.sin(yaw) * local_target[1]
    y = math.sin(yaw) * -local_target[0] + math.cos(yaw) * local_target[1]
    
    return (x, y)


def estimate_velocity(obj_pos_history):
    if len(obj_pos_history) <= 5:
        return
    
    pos1 = obj_pos_history[-5]
    pos2 = obj_pos_history[-1] #skip one frame

    if math.degrees(abs(pos1[3] - pos2[3])) > 5:
        return

    time_diff = pos2[4] - pos1[4]
    velocity_vector = ((pos2[0] - pos1[0]) / -time_diff, (pos2[1] - pos1[1]) / time_diff, (pos2[2] - pos1[2]) / time_diff)

    msg = vehicle.message_factory.vision_speed_estimate_encode(
        0, #time boot
        velocity_vector[1], velocity_vector[0], velocity_vector[2],
        [float("NAN"), 0, 0, 0, 0, 0, 0, 0, 0], #covariance
        0, # reset counter
    )
    vehicle.send_mavlink(msg)

    return velocity_vector


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
    print(("SAFE: " if avg_deviation < 0.05 else "UNSAFE: ") + str(avg_deviation))
    return avg_deviation < 0.05


def land_on_object(rate):

    yaw = vehicle.attitude.yaw
    start_pos = vehicle.location.global_frame

    print ("landing on object")
    tolerance = 0.25

    def within_tolerance(multiplier=1):
        return abs(relative_coordinates[0]) < tolerance * multiplier and abs(relative_coordinates[1]) < tolerance * multiplier

    pos_buffer = []

    #center bag and maintain altitude
    final_descent = False
    final_descent_height = 0.5
    centered_frames = 0
    centered = False
    while not final_descent and vehicle.armed:
        object_coordinates = get_bag_image_coords()
        if object_coordinates == None:
            print("target lost")
            send_frame_velocity(0, 0, -0.3, yaw)
            #vehicle.airspeed = 1
            #vehicle.simple_goto(start_pos)
            rate.sleep()
            continue

        target_lost = False
        alt = vehicle.rangefinder.distance

        pixel_coordinates = object_coordinates[1]
        relative_coordinates = object_coordinates[0]
        #if alt < 1:
            #relative_coordinates = (relative_coordinates[0] - 0.2, relative_coordinates[1] + 0.2)
        local_coordinates = pixel_to_world((pixel_coordinates[0], pixel_coordinates[1], alt))
        world_coordinates = rotate_to_global((local_coordinates[0], local_coordinates[1]))
        pos_buff.append((world_coordinates[0], world_coordinates[1], alt, vehicle.attitude.yaw, time.time()))
        if len(pos_buff) > 6:
            pos_buff.pop(0)
        estimate_velocity(pos_buff)

        if within_tolerance() and centered_frames >= 8:
            if not centered and alt > 2:
                #brake_for(1, yaw)
                centered = True
            if alt < final_descent_height:
                final_descent = True
                print ("final descent")
                break

        adjustment_speed = max (1, min(2, alt / 2))

        if not within_tolerance():
            centered_frames = 0
            centered = False
            print ("uncentered: " + str(alt) + " " + str(object_coordinates))
            send_frame_velocity (relative_coordinates[1] * adjustment_speed, relative_coordinates[0] * adjustment_speed, -0.2 if alt < final_descent_height else 0, yaw)
        else:
            centered_frames += 1
            print ("centered: " + str(alt) + " " + str(object_coordinates))
            send_frame_velocity (relative_coordinates[1] * adjustment_speed, relative_coordinates[0] * adjustment_speed, 0.2 if alt > 2 else (0.1 if alt > final_descent_height * 0.8 else -0.2), yaw)

        rate.sleep()

    if not final_descent:
        print ("landing attempt failed")
        vehicle.airspeed = 1
        #vehicle.simple_goto(start_pos)
        return

    set_gripper(False)
    count = 0
    while count < 6:
        send_frame_velocity (0, 0, 0.3, yaw)
        time.sleep(0.5)
        count += 1
    set_gripper(True)

    vehicle.armed = False
    time.sleep(5)
    return


pos_buff = []
def camera_callback (img):
    global down_frame
    global pos_buff
    down_frame = bridge.imgmsg_to_cv2 (img)
    """if vehicle is not None:
        bag = get_bag_image_coords()
        if bag is not None:
            coord = bag[1]
            print (math.degrees(vehicle.attitude.yaw))
            local_coordinates = pixel_to_world(coord[0], coord[1], vehicle.rangefinder.distance)
            world_coordinates = rotate_to_global((local_coordinates.x, local_coordinates.y))
            pos_buff.append((world_coordinates[0], world_coordinates[1], vehicle.rangefinder.distance, vehicle.attitude.yaw, time.time()))
            if len(pos_buff) > 6:
                pos_buff.pop(0)
            vel = estimate_velocity(pos_buff)
            if vel is not None and (abs(vel[0]) > 0.1 or abs(vel[1]) > 0.1):
                print(f"Velocity: {int(vel[0] * 1000) / 1000.0}, {int(vel[1] * 1000) / 1000.0}")"""


def depth_callback(img):
    global down_depth
    down_depth = bridge.imgmsg_to_cv2(img)
    #check_safe_landing()


# Callback function for vehicle's 'armed' attribute
def arm_callback(self, attr_name, value):
    state_pub.publish(value)
    print ("callback " + str(value))

def world_to_pixel(world_pos):
    _intrinsics = rs2.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = rs2.distortion.none  
    _intrinsics.coeffs = [i for i in cameraInfo.D]

    x, y = rs2.rs2_project_point_to_pixel(_intrinsics, (world_pos[0], world_pos[1], world_pos[2]))
    return (x, y)

def pixel_to_world(pixel_pos):
    _intrinsics = rs2.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = rs2.distortion.none  
    _intrinsics.coeffs = [i for i in cameraInfo.D]

    x, y, z = rs2.rs2_deproject_pixel_to_point(_intrinsics, (pixel_pos[0], pixel_pos[1]), pixel_pos[2])
    return (x, y)

def recieve_camera_info(msg):
    global cameraInfo
    cameraInfo = msg

def do_misson():
    cmds = vehicle.commands
    cmds.clear()
    cmd2=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, 29.970872, -95.637599, 3)
    cmd3=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, 29.970750, -95.637597, 3)
    cmd4=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, 29.970872, -95.637599, 3)
    cmd5=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, 29.970750, -95.637597, 3)
    cmd6=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, 29.970872, -95.637599, 3)
    cmd7=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, 29.970750, -95.637597, 3)
    cmds.add(cmd2)
    cmds.add(cmd3)
    cmds.add(cmd4)
    cmds.add(cmd5)
    cmds.add(cmd6)
    cmds.add(cmd7)
    vehicle.airspeed = 0.5
    vehicle.groundspeed = 0.5
    print("uploading...")
    cmds.upload() # Send commands

    takeoff(2)
    vehicle.simple_goto(vehicle.location.global_relative_frame)

    vehicle.mode = VehicleMode("AUTO")
    if vehicle.mode != VehicleMode("AUTO"):
        print("failed to switch to auto")
    
def start_node():
    global object_detector
    object_detector = BlobDetection()
    print ("model loaded")

    rospy.Subscriber('/down/color/image_raw', Image, camera_callback, queue_size=1)
    rospy.Subscriber('/down/depth/image_rect_raw', Image, depth_callback, queue_size=1)
    rospy.Subscriber('/front/color/camera_info', CameraInfo, recieve_camera_info)
    global state_pub
    state_pub = rospy.Publisher('is_armed', Bool, queue_size=1)

    global bridge
    bridge = CvBridge()

    rate = rospy.Rate(20)

    # Wait for rospy
    while rospy.is_shutdown():
        rate.sleep()

    global vehicle
    vehicle = connect('127.0.0.1:14540', baud=921600, wait_ready=True)

    # Add arm_callback as a listener for 'armed' attribute changes
    vehicle.add_attribute_listener('armed', arm_callback)

    print("connected")

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ("Waiting for vehicle to initialise...")
        time.sleep(1)

    launch_position = vehicle.location.global_relative_frame
    set_gripper(True)

    while not rospy.is_shutdown():
        
        if not vehicle.armed or not vehicle.mode == VehicleMode("GUIDED"):
            print ("Waiting for arming... ")
            mission_started = False
            time.sleep(1)
        elif not mission_started:
            mission_started = True
            do_misson()
        
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("offboard_node")
    rospy.loginfo("offbard node started")
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass 
