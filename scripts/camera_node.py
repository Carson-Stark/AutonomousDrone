#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

width = 1920
height = 1080
fps = 30

def start_capture ():
    right_pub = rospy.Publisher('rightcam', Image, queue_size=1)
    left_pub = rospy.Publisher('leftcam', Image, queue_size=1)
    down_pub = rospy.Publisher('downcam', Image, queue_size=1)
    bridge = CvBridge()

    cap = cv2.VideoCapture(f"nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){width}, height=(int){height}," + \
                f"format=(string)NV12, framerate=(fraction){fps}/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink drop=true", cv2.CAP_GSTREAMER)
    # Check if camera opened successfully
    if (cap.isOpened() == False):
        print("Error opening video file")
        return

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

    while cap.isOpened() and not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            front_left = frame[int(height/2.0):height,0:int(width/2.0)]
            front_right = frame[int(height/2.0):height,int(width/2.0):width]
            down_left = frame[:int(height/2.0),0:int(width/2.0)]
            front_left_msg = bridge.cv2_to_imgmsg(front_left, "bgr8")
            front_right_msg = bridge.cv2_to_imgmsg(front_right, "bgr8")
            down_left_msg = bridge.cv2_to_imgmsg(down_left, "bgr8")

            left_pub.publish(front_left_msg)
            right_pub.publish(front_right_msg)
            down_pub.publish(down_left_msg)

            cv2.waitKey(10)

        else:
            break
    
    cap.release()

if __name__ == '__main__':
    rospy.init_node('camera_node', anonymous=True)
    rospy.loginfo("camera node started")
    try:
        start_capture()
    except rospy.ROSInterruptException:
        pass