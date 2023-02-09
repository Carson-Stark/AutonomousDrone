#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

import time

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg
    RecordTrigger.armed = current_state.armed

def position_callback(data):
    global current_position
    current_position = data

def reached_target (target, tolerance=0.2):
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
    local_pos_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection and arm
    while rospy.is_shutdown() or not current_state.connected:
        rate.sleep() 
        
    poseTakeoff = PoseStamped()
    poseTakeoff.pose.position.x = 0
    poseTakeoff.pose.position.y = 0
    poseTakeoff.pose.position.z = 4

    poseA = PoseStamped()
    poseA.pose.position.x = 0
    poseA.pose.position.y = 10
    poseA.pose.position.z = 4

    poseB = PoseStamped()
    poseB.pose.position.x = 0
    poseB.pose.position.y = -10
    poseB.pose.position.z = 4

    # Send a few setpoints before starting
    for i in range(100):   
        if rospy.is_shutdown():
            break

        local_pos_pub.publish(poseTakeoff)
        rate.sleep()

    while not rospy.is_shutdown() and current_state.connected:

        if not current_state.armed:
            local_pos_pub.publish(poseTakeoff)
            rate.sleep()
        elif current_state.mode == "OFFBOARD":
            fly_to_target (poseA, rate)
            fly_to_target (poseB, rate)

        rate.sleep()
