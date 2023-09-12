#!/bin/bash
echo "Starting camera 1"
gnome-terminal -- roslaunch realsense2_camera rs_camera.launch serial_no:=819112073352 camera:=front
#echo "Starting camera 2"
#gnome-terminal -- roslaunch realsense2_camera rs_camera.launch serial_no:=918512072489 camera:=down
