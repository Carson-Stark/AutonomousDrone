from dronekit import connect, VehicleMode, mavutil, LocationGlobal, LocationGlobalRelative, Command
from drone.srv import Coordinates
import pyrealsense2 as rs2
import math
import time

log = print

def rangefinder_distance(vehicle):
    return vehicle.rangefinder.distance
    #return vehicle.location.global_relative_frame.alt

def takeoff(vehicle, targetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    start_yaw = -1

    if not vehicle.armed:
        try:
            log("Arming...")
            vehicle.arm(True, 5)
            time.sleep(5)
        except:
            log("ARM FAILED")
            return False
        
    
    log ("Taking off to " + str(targetAltitude))

    if rangefinder_distance(vehicle) > 0.5:
        launch_pos = vehicle.location.global_relative_frame
        target_pos = LocationGlobalRelative(launch_pos.lat, launch_pos.lon, targetAltitude)
        vehicle.simple_goto(target_pos)
    else:
        vehicle.simple_takeoff(targetAltitude)
    
    # Wait until the vehicle reaches a safe height
    while vehicle.armed and vehicle.mode == VehicleMode("GUIDED"):
        log (" Altitude: " + str(vehicle.location.global_relative_frame.alt))

        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= targetAltitude*0.95:
            log ("Reached target altitude")
            send_frame_velocity(vehicle, 0, 0, 0, start_yaw)
            return True

        if (abs(math.degrees(vehicle.attitude.roll)) > 15 or abs(math.degrees(vehicle.attitude.pitch)) > 15) and vehicle.location.global_relative_frame.alt < 2:
            print("takeoff failsafe triggered")
            terminate_flight(vehicle)
            return False

        if vehicle.location.global_relative_frame.alt > 3:
            start_yaw = vehicle.attitude.yaw

        if start_yaw != -1 and abs(math.degrees(vehicle.attitude.yaw - start_yaw)) > 30:
            print("takeoff yaw unstable")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(0.5)
            return False

        time.sleep(0.5)

    return False

# velocity_x > 0 => fly foward
# velocity_x < 0 => fly backward
# velocity_y > 0 => fly right
# velocity_y < 0 => fly left
# velocity_z < 0 => ascend
# velocity_z > 0 => descend
def send_frame_velocity(vehicle, velocity_x, velocity_y, velocity_z, yaw):
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
    if yaw != 0:
        set_yaw(vehicle, yaw)

def turn_towards(vehicle, yaw, target_alt):
    log("turning to heading: " + str(convert180(yaw)))
    print(yaw)
    while vehicle.armed and abs(math.degrees(vehicle.attitude.yaw) - convert180(yaw)) > 3:
        alt = vehicle.location.global_relative_frame.alt
        #if alt < target_alt * 0.8:
        #    send_frame_velocity(vehicle, 0, 0, -0.3, yaw)
        #elif alt > target_alt * 1.2:
        #    send_frame_velocity(vehicle, 0, 0, 0.3, yaw)
        #else:
        send_frame_velocity(vehicle, 0, 0, 0, yaw)
        #set_yaw(vehicle, yaw)
        time.sleep(0.1)

def send_velocity_for(vehicle, x, y, z, seconds):
    yaw = vehicle.attitude.yaw
    sleep_increment = 0.2
    count = 0
    while count < seconds:
        send_frame_velocity(vehicle, x, y, z, yaw)
        time.sleep(sleep_increment)
        count += sleep_increment

def terminate_flight(vehicle):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, #command
        0, #confirmation
        0, # arm disarm
        21196, # force
        0, 0, 0, 0, 0)
    
    # send command to vehicle
    vehicle.send_mavlink(msg)

def set_yaw(vehicle, heading, speed=360):
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
        speed,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    
    # send command to vehicle
    vehicle.send_mavlink(msg)

def set_gripper(vehicle, closed):
    global gripper_status
    msg = vehicle.message_factory.command_long_encode(
		0, 0, 
		mavutil.mavlink.MAV_CMD_DO_GRIPPER,
		0,
		1, #servo number
		1 if closed else 0, #open/closes
		0,0,0,0,0
		)
    print (closed)
    vehicle.send_mavlink(msg)
    gripper_status = closed

def get_gripper_status():
    return gripper_status

def distance_meters(vehicle, target):
    return (abs(vehicle.location.global_relative_frame.lat - target.lat) * 111139, abs(vehicle.location.global_relative_frame.lon - target.lon) * 111139)

#account for roll and pitch in image coordinates
def get_offsets(vehicle, frame, camera_info):
    roll = vehicle.attitude.roll
    pitch = vehicle.attitude.pitch

    alt = vehicle.location.global_relative_frame.alt
    if alt > 4:
        return (0, 0)
    
    # Calculate offsets using trigonometry
    horizontal_offset = alt * math.tan(roll)
    vertical_offset = alt * math.tan(pitch)

    pixel_coord = world_to_pixel((horizontal_offset, vertical_offset, alt), camera_info)
    return (-(pixel_coord[0] - (frame.shape[1] / 2)), -(pixel_coord[1] - (frame.shape[0] / 2)))

def convert180(angle):
    if angle > 180:
        angle -= 360
    return angle

def sign(num):
    return 1 if num > 0 else -1

#(X = right, Y = foward)
def rotate_to_global (vehicle, local_target):
    yaw = -vehicle.attitude.yaw

    x = math.cos(yaw) * local_target[0] + math.sin(yaw) * local_target[1]
    y = math.sin(yaw) * -local_target[0] + math.cos(yaw) * local_target[1]
    
    return (x, y)

def get_heading(vehicle, waypoint):
    curr_location = vehicle.location.global_relative_frame

    initial_bearing = math.degrees(math.atan2(waypoint.lon - curr_location.lon, waypoint.lat - curr_location.lat))
    print(f"{curr_location.lat} {curr_location.lon} {waypoint.lon} {waypoint.lat}")
    print (initial_bearing)

    # Normalize the initial_bearing to the range [-180, 180)
    adjusted_bearing = (initial_bearing + 360) % 360

    return adjusted_bearing

def waypoint_relative(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)

    return LocationGlobalRelative(newlat, newlon, alt)

def set_mission_speed(speed):
    msg = vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            1, #type
            speed, #speed
            0,0,0,0,0,0
        )
    vehicle.send_mavlink(msg)

def world_to_pixel(world_pos, cameraInfo):
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

def pixel_to_world(pixel_pos, cameraInfo):
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
