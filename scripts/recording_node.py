#!/usr/bin/env python3
import cv2
import datetime
import os
from time import sleep
import rospy
from mavros_msgs.msg import State
from threading import Thread

current_state = State()

def state_cb(msg):
    global current_state
    if not current_state.armed and msg.armed:
        recorder.start_recording()
    elif current_state.armed and not msg.armed:
        recorder.stop_recording()
    current_state = msg

class Recorder:

    def __init__ (self, width, height, fps):
        self.width = width
        self.height = height
        self.fps = fps
        self.recording = False

    def capture (self):
        print ("recording started")
        self.recording = True

        #start video capture
        cap = cv2.VideoCapture(f"nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){self.width}, height=(int){self.height}," + \
                    f"format=(string)NV12, framerate=(fraction){self.fps}/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink drop=true", cv2.CAP_GSTREAMER)

        #start video writer
        timestamp = datetime.datetime.now()
        path = os.path.expanduser(f'~/Desktop/Flight Recordings/recording{timestamp}.mp4')
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'XVID'), self.fps, (int(self.width/2.0), self.height))

        # Check if camera opened successfully
        if not cap.isOpened() or not writer.isOpened():
            print("Error opening video file")
            return

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 5)

        #record until disarmed
        while cap.isOpened() and writer.isOpened() and self.recording:
            ret, frame = cap.read()
            if ret:
                front_left = frame[int(self.height/2.0):self.height,0:int(self.width/2.0)]
                down_left = frame[:int(self.height/2.0),0:int(self.width/2.0)]

                frame = cv2.vconcat ((front_left, down_left))
                writer.write (frame)

                cv2.waitKey(50)
            else:
                break

        print ("recording stopped")
        
        cap.release()
        writer.release()

    def start_recording(self):
        Thread(target=self.capture, args=(), daemon=True).start()
        return self

    def stop_recording(self):
        self.recording = False

recorder = None

def start_node ():
    rospy.Subscriber("mavros/state", State, callback = state_cb)
    global recorder
    recorder = Recorder(1920, 1080, 30)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('recording_node', anonymous=True)
    rospy.loginfo("recording node started")
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass