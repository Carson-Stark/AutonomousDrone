#!/bin/bash
echo "Starting camera 1"
gnome-terminal -- roslaunch realsense2_camera rs_camera.launch serial_no:=819112073352 camera:=front color_width:=640 color_height:=480
echo "Starting camera 2"
gnome-terminal -- roslaunch realsense2_camera rs_camera.launch serial_no:=918512072489 camera:=down color_width:=640 color_height:=480
