#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
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

def reached_target (target, tolerance=0.1):
    return abs(current_position.pose.position.x - target.pose.position.x) < tolerance and abs(current_position.pose.position.y - target.pose.position.y) < tolerance \
            and abs(current_position.pose.position.z - target.pose.position.z) < tolerance

def hold_position (target, hold_time, rate):
    start_time = time.time()
    while (time.time() - start_time) < hold_time:
        local_pos_pub.publish(target)
        rate.sleep()

def fly_to_target (target, rate):
    while not reached_target(target):
        if rospy.is_shutdown() or not current_state.armed:
            return
        local_pos_pub.publish(target)
        rate.sleep()

    hold_position (target, 3, rate)

if __name__ == "__main__":
    rospy.init_node("offboard_node")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, position_callback)
    global local_pos_pub
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while not rospy.is_shutdown() and not current_state.connected and not current_state.armed:
        rate.sleep()

    poseTakeoff = PoseStamped()
    poseTakeoff.pose.position.x = 0
    poseTakeoff.pose.position.y = 0
    poseTakeoff.pose.position.z = 3

    poseA = PoseStamped()
    poseA.pose.position.x = 5
    poseA.pose.position.y = 0
    poseA.pose.position.z = 3

    poseB = PoseStamped()
    poseB.pose.position.x = -5
    poseB.pose.position.y = 0
    poseB.pose.position.z = 3

    # Send a few setpoints before starting
    for i in range(100):   
        if rospy.is_shutdown():
            break

        local_pos_pub.publish(poseTakeoff)
        rate.sleep()

    while not rospy.is_shutdown() and current_state.armed:

        if current_state.mode == "OFFBOARD":
            fly_to_target (poseA, rate)
            fly_to_target (poseB, rate)

        rate.sleep()