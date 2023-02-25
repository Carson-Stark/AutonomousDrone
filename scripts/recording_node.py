#!/usr/bin/env python3
import cv2
import datetime
import os
import time
import rospy

class RecordTrigger:
    bool armed = False

width = 1920
height = 1080
fps = 20

writer = None
current_state = State()

def state_cb(msg):
    global current_state
    global writer
    if not current_state.armed and msg.armed:
        writer = start_recording ()
    elif current_state.armed and not msg.armed:
        print ("stopped recording")
        writer = None

    current_state = msg

def start_recording ():
    print ("recording started")

    #start video capture
    cap = cv2.VideoCapture(f"nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){width}, height=(int){height}," + \
                f"format=(string)NV12, framerate=(fraction){fps}/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink drop=true", cv2.CAP_GSTREAMER)

    #start video writer
    timestamp = datetime.datetime.now()
    path = os.path.expanduser(f'~/Desktop/Flight Recordings/recording{timestamp}.mp4')
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'XVID'), fps, (int(width/2.0), height))

    # Check if camera opened successfully
    if not cap.isOpened() or not writer.isOpened():
        print("Error opening video file")
        return

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 5)

    #record until disarmed
    while cap.isOpened() and writer.isOpened() and RecordTrigger.armed:
        ret, frame = cap.read()
        if ret:
            front_left = frame[int(height/2.0):height,0:int(width/2.0)]
            down_left = frame[:int(height/2.0),0:int(width/2.0)]

            frame = cv2.vconcat ((front_left, down_left))
            writer.write (frame)

            cv2.waitKey(50)
        else:
            break

    print ("recording stopped")
    
    cap.release()
    writer.release()

def wait_for_armed ():
    while True:
        #wait for armed
        if RecordTrigger.armed:
            start_recording()

        sleep(1)

if __name__ == '__main__':
    rospy.init_node('recording_node', anonymous=True)
    rospy.loginfo("recording node started")
    try:
        wait_for_armed()
    except rospy.ROSInterruptException:
        pass