#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Header
from drone.srv import Coordinates, CoordinatesResponse
import message_filters

from threading import Thread
import datetime
import os
import numpy as np

import subprocess, shlex, psutil
import time

armed = False

def state_cb(value):
    global armed
    global cap
    armed = value.data
    if armed and not cap.recording:
        cap.start_recoding()
    elif not armed and cap.recording:
        cap.stop_recording()

class CamCapture:
    def __init__(self, width, height, fps):
        self.width = width
        self.height = height
        self.fps = fps
        self.recording = False
        self.ret = False
        self.use_recording = False

    def start_recoding(self):
        print ("start recording")
        timestamp = datetime.datetime.now()
        path = os.path.expanduser(f'~/Desktop/FlightRecordings/recording{timestamp}.mp4')
        self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'XVID'), self.fps, (int(self.width), self.height))
        self.recording = True
        Thread(target=self.record, args=(), daemon=True).start()

    def stop_recording(self):
        print ("stop recording")
        self.recording = False

    def record(self):
        while not rospy.is_shutdown() and self.recording:
            try:
                self.writer.write (frame)
            except:
                pass

        self.writer.release()

def image_frame_callback(fc, fd, dc, dd):
    fc_img = cv2.cvtColor(bridge.imgmsg_to_cv2(fc), cv2.COLOR_RGB2BGR)
    fd_img = bridge.imgmsg_to_cv2(fd)
    fd_8bit = cv2.convertScaleAbs(fd_img, alpha=255.0 / 8000)
    fd_img = cv2.cvtColor(cv2.applyColorMap(fd_8bit, cv2.COLORMAP_JET), cv2.COLOR_RGB2BGR)
    dc_img = cv2.cvtColor(bridge.imgmsg_to_cv2(dc), cv2.COLOR_RGB2BGR)
    dd_img = bridge.imgmsg_to_cv2(dd)
    dd_8bit = cv2.convertScaleAbs(dd_img, alpha=255.0 / 8000)
    dd_img = cv2.cvtColor(cv2.applyColorMap(dd_8bit, cv2.COLORMAP_JET), cv2.COLOR_RGB2BGR)

    global frame
    front = np.concatenate((fc_img, fd_img), axis=0)
    down = np.concatenate((dc_img, dd_img), axis=0)
    frame = np.concatenate((front, down), axis=1)
    frame = cv2.resize(frame, (1920, 1080), interpolation= cv2.INTER_LINEAR)

    #detector.detectWhite(front, 10000)

def start_node():
    global bridge
    bridge = CvBridge()
    state_sub = rospy.Subscriber("is_armed", Bool, callback=state_cb)
    front_color = message_filters.Subscriber("front/color/image_raw", Image)
    front_depth = message_filters.Subscriber("front/depth/image_rect_raw", Image)
    down_color = message_filters.Subscriber("down/color/image_raw", Image)
    down_depth = message_filters.Subscriber("down/depth/image_rect_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer([front_color, front_depth, down_color, down_depth], queue_size=5, slop=0.1)
    ts.registerCallback(image_frame_callback)

    #global detector
    #detector = BlobDetection()

    global cap
    cap = CamCapture(1920, 1080, 10)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('camera_node', anonymous=True)
    rospy.loginfo("camera node started")
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass 
