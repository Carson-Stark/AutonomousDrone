#!/usr/bin/env python3

import os
import cv2
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess

fps = 5
width = 480
height = 270

def start_node():
    cv_file = cv2.FileStorage()
    script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
    map_file_path = os.path.join(script_dir, 'stereoMap.xml')
    print (map_file_path)
    cv_file.open(str(map_file_path), cv2.FileStorage_READ)

    global bridge
    bridge = CvBridge()
    
    global out
    out = cv2.VideoWriter(f'appsrc ! videoconvert' + \
    ' ! x264enc speed-preset=ultrafast' + \
    ' ! rtspclientsink location=rtsp://172.25.20.179:8554/mystream',
    cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
    if not out.isOpened():
        raise Exception("can't open video writer")
    
    rospy.Subscriber('/front/color/image_raw', Image, recieve_frame)

    rospy.loginfo("serving image to RTSP server...")

    rospy.spin()

    cv2.destroyAllWindows()

def recieve_frame(img):
    cam_img = cv2.resize (bridge.imgmsg_to_cv2(img), (width, height))
    cam_img = cv2.cvtColor(cam_img, cv2.COLOR_RGB2BGR)
    out.write(cam_img)


if __name__ == '__main__':
    rospy.init_node('video_stream_node', anonymous=True)
    rospy.loginfo("video stream node started")

    try:
        start_node()
    except rospy.ROSInterruptException:
        pass