#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

def start_capture (width, height, fps):
    right_pub = rospy.Publisher('rightcam', Image, queue_size=1)
    left_pub = rospy.Publisher('leftcam', Image, queue_size=1)
    bridge = CvBridge()

    cap = cv2.VideoCapture(f"nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){width}, height=(int){height}," + \
                f"format=(string)NV12, framerate=(fraction){fps}/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink drop=true", cv2.CAP_GSTREAMER)
    # Check if camera opened successfully
    if (cap.isOpened() == False):
        print("Error opening video file")
        return

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

    while not rospy.is_shutdown():
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                front_left = frame[int(height/2.0):height,0:int(width/2.0)]
                front_right = frame[int(height/2.0):height,int(width/2.0):width]
                front_left_msg = bridge.cv2_to_imgmsg(front_left, "bgr8")
                front_right_msg = bridge.cv2_to_imgmsg(front_right, "bgr8")

                left_pub.publish(front_left_msg)
                right_pub.publish(front_right_msg)
            else:
                break

if __name__ == '__main__':
    rospy.init_node('camera_node', anonymous=True)
    rospy.loginfo("camera node started")
    try:
        start_capture(1920, 1080, 30)
    except rospy.ROSInterruptException:
        pass