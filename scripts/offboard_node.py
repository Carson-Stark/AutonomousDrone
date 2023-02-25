#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

import time

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def position_callback(data):
    global current_position
    current_position = data

def reached_target (target, tolerance=0.2):
    return abs(current_position.pose.position.x - target.pose.position.x) < tolerance and abs(current_position.pose.position.y - target.pose.position.y) < tolerance \
            and abs(current_position.pose.position.z - target.pose.position.z) < tolerance

def hold_position (target, hold_time, rate):
    start_time = time.time()
    while (time.time() - start_time) < hold_time:
        goal_pub.publish(target)
        rate.sleep()

def fly_to_target (target, timeout, rate, hold=0):
    start_time = time.time()
    while not reached_target(target, 0.5) and (time.time() - start_time) < timeout:
        if rospy.is_shutdown() or not current_state.armed:
            return
        goal_pub.publish(target)
        rate.sleep()

    hold_position (target, hold, rate)

def spiral (center, radius, lane_size, altitude, rate):

    fly_to_target (center, 10, rate, 3)
    
    if lane_size == 0:
        return [center]

    current_radius = 0
    direction = 0  # 0: right, 1: up, 2: left, 3: down
    centerX, centerY = center.pose.position.x, center.pose.position.y
    x, y = centerX, centerY
    direction_changed = False
    while current_radius < radius:
        direction_changed = False
        if direction == 0:  # move right
            x += lane_size
            if x - centerX > current_radius + lane_size:
                x = centerX + current_radius + lane_size
                direction = 1
                direction_changed = True
        elif direction == 1:  # move up
            y += lane_size
            if y - centerY > current_radius + lane_size:
                y = centerY + current_radius + lane_size
                direction = 2
                direction_changed = True
        elif direction == 2:  # move left
            x -= lane_size
            if centerX - x > current_radius + lane_size:
                x = centerX - current_radius - lane_size
                direction = 3
                direction_changed = True
        else:  # move down
            y -= lane_size
            if centerY - y > current_radius + lane_size:
                y = centerY - current_radius - lane_size
                direction = 0
                current_radius += lane_size
                direction_changed = True
        
        if direction_changed:
            waypoint = PoseStamped()
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.position.z = altitude
            fly_to_target (waypoint, 5 + current_radius * 2, rate)

def land_on_object(object_coordinates, rate):
    target_velocity = TwistStamped()
    target_velocity.twist.linear.z = -0.3
    target_velocity.twist.linear.x = 0
    target_velocity.twist.linear.y = 0
    target_pos = PoseStamped()
    target_pos.pose.position.x = object_coordinates[0]
    target_pos.pose.position.y = object_coordinates[1]
    target_pos.pose.position.z = object_coordinates[2]
    while current_position.pose.position.z > 0.5:
        vel_pub.publish (target_velocity)
        #local_position_pub.publish (target_pos)
    target_velocity = TwistStamped()
    while not rospy.is_shutdown():
        target_velocity.twist.linear.z = -0.1
        vel_pub.publish (target_velocity)
    

if __name__ == "__main__":
    rospy.init_node("offboard_node")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, position_callback)
    global local_position_pub
    global vel_pub
    global goal_pub
    local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10) 
    vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while rospy.is_shutdown() or not current_state.connected or not current_state.armed:
        rate.sleep() 
        
    poseTakeoff = PoseStamped()
    poseTakeoff.pose.position.x = 0
    poseTakeoff.pose.position.y = 0
    poseTakeoff.pose.position.z = 10

    # Send a few setpoints before starting
    for i in range(100):   
        if rospy.is_shutdown():
            break

        goal_pub.publish(poseTakeoff)
        rate.sleep()

    while not rospy.is_shutdown() and current_state.armed:

        if current_state.mode == "OFFBOARD":
            spiral(poseTakeoff, 4, 3, 10, rate)
            land_on_object ((5, 5, 0), rate)
            break

        rate.sleep()
