# Autonomous Drone Project

<img width="1920" height="1080" alt="1" src="https://github.com/user-attachments/assets/b5b19692-d7d6-4e2d-8c27-31f25cdca4c1" />

## Project Overview

This project implements an autonomous drone system integrating robotics, computer vision, and machine learning technologies. It features obstacle avoidance, precision landing using custom visual servoing, object detection deployed on edge devices, and payload verification with depth sensing. The system supports cellular connectivity for extended operational range and includes a custom mobile app for remote monitoring and mission control. Core hardware includes Jetson Nano and Pixhawk autopilot, with software components built on ROS, DroneKit, and TensorFlow Lite.

### Features

- Forward and downward obstacle avoidance and obstacle-aware path planning
- Object detection model trained on custom dataset and deployed on edge
- Custom visual servoing algorithm for reliable precision landing on a target
- Payload verification with downward depth camera
- Cellular connectivity enabling unlimited operable range
- Custom mobile app for remote monitoring and operation
- Summon drone or cue mission with GPS coordinates from mobile device
- Custom 3D printed gripper mechanism with servo actuation
- Safe landing detection with downward depth camera

### Project Timeline

**Started:** December 2022

**Completed:** December 2024

### YouTube Video

*Follow the complete development journey from start to finish*

#### I Spent Two Years Building and Autonomous Drone...
[![Can I Build a TikTok Clone in 24 Hours?](https://img.youtube.com/vi/B96JGoWQOsE/0.jpg)](https://www.youtube.com/watch?v=B96JGoWQOsE)

## Flight Platform

### Hardware

- **Standard S500 Hexacopter Frame** with 3D-printed mounts for companion computer and sensors
- **Pixhawk Flight Controller** for low-level stabilization, GPS navigation, and built-in flight functions like return-to-home and loiter, connected to companion computer via serial
- **Jetson Nano** companion computer for autonomous decision making
- **Coral TPU** for realtime object detection inference
- **6x Brushless Motors + ESCs** (T-Motor)
- **Intel RealSense Depth Cameras** (forward-facing for obstacle detection, downward-facing for landing validation)
- **4G LTE Modem** for remote connectivity and telemetry streaming
- **3D-printed Gripper Mechanism** powered by servo for bag pickup
- **9000mAh Lipo Battery** powered all onboard electronics including the companion computer (with voltage regulator), provided 10+ minutes of flight time

### Software
- **ArduPilot Firmware (ArduCopter)**: Used after switching from PX4 for greater reliability and ease of integration
- **DroneKit-Python SDK**: Used for high-level drone control and automation sequences
- **ROS Melodic**: coordinated communication between various python nodes running simultaneously, originally used for mavros and Gazebo with PX4
- **OpenCV**: Powered all computer vision algorithms, from depth map generation to custom image-based target tracking
- **TF-Lite**: Trained EfficientDet-Lite model on a custom dataset collected from actual drone footage, optimized for aerial bag detection with Coral TPU acceleration

## Installation

### Prerequisites

Ensure you have the following prerequisites installed and configured before proceeding:

- Ubuntu 18.04 or later (recommended for ROS compatibility)
- ROS Melodic or Noetic installed (see [ROS installation guide](http://wiki.ros.org/ROS/Installation))
- Python 3.6 or higher
- Jetson Nano setup with required drivers and libraries (refer to NVIDIA Jetson setup documentation)
- Intel RealSense SDK installed (see [Intel RealSense SDK installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md))
- DroneKit-Python installed:
  ```bash
  pip3 install dronekit
  ```
- TensorFlow Lite runtime installed for object detection:
  ```bash
  pip3 install tflite-runtime
  ```
- Coral Edge TPU device attached for hardware-accelerated object detection.
- Edge TPU runtime installed (libedgetpu). Follow the official Coral setup guide: https://coral.ai/docs/setup/
- PyCoral Python library installed for interfacing with the Edge TPU:
  ```bash
  pip3 install pycoral
  ```
- Other dependencies listed in `package.xml` and `requirements.txt` (will be installed in later steps)

### Software Setup

This project is a ROS package and should be installed into a ROS workspace alongside other packages.

1. Setup Intel RealSense camera and drivers:

   - Download and install the RealSense ROS1 wrapper package into your workspace by following the instructions here: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy
   - Note: `depth_node.py` is a fork of [this script](https://github.com/thien94/vision_to_mavros/blob/master/scripts/d4xx_to_mavlink.py), modified to accept ROS messages from the RealSense wrapper instead of using `pyrealsense2`.
   - For detailed RealSense camera setup with ArduPilot, refer to: https://ardupilot.org/copter/docs/common-realsense-depth-camera.html (manual setup section)

2. Clone this repository into the `src` directory of your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Carson-Stark/AutonomousDrone.git
   cd ..
   ```

3. Install ROS dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the workspace:
   ```bash
   catkin_make
   source devel/setup.bash
   ```

5. Install Python dependencies:
   ```bash
   pip3 install -r src/AutonomousDrone/requirements.txt
   ```

6. Setup MavProxy for MAVLink communication.

   ```bash
   pip3 install MAVProxy
   ```

   - Replace the placeholders in `start_telemetry_forwarding.sh` with your actual device and IP addresses:
   - Replace `your_sudo_password` with your system sudo password.
   - Replace `/dev/your_serial_device` with your actual serial device, e.g., `/dev/ttyTHS1`.
   - Replace `<IP_ADDRESS_1>` and `<IP_ADDRESS_2>` with the IP addresses of the devices you want to forward MAVLink data to.

7. Setup `tcp_server.py` with your machines IP. Check out [ZeroTier VPN](https://www.zerotier.com/) to add your devices to a virtual LAN network.

### Hardware Setup

- Connect the Pixhawk autopilot to your offboard system via Telemetry port and UART serial connection. See detailed instructions here for RaspberryPI: https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html 
- Connect the Coral TPU to your companion computer via USB for fast TFlite inference.
- Configure the autopilot using Mission Planner or QGroundControl.
- Calibrate sensors and set up flight parameters as per your drone hardware.
- Ensure stable internet connection with WiFi or cellular modem so the companion computer can accept TCP messages.

## Usage

1. Power on drone and companion computer.

2. Run via ssh or setup to run automatically on boot:
   ```
   roslaunch drone drone.launch
   ```

3. Verify the camera, offboard, and tcp_server nodes are initialized and running

4. Send mission commands with the custom mobile app or via another TCP client.

## Project Structure

### Launch Files

- `launch/drone.launch`: Launches the full autonomous drone stack including camera nodes, object detection, and flight control.
- `launch/camera.launch`: Launches camera and recording nodes only.

### Scripts

- `scripts/offboard_node.py`: Main offboard control node for autonomous flight commands.
- `scripts/mission_definitions.py`: Creates action sequences for a variety of preset mission types.
- `scripts/camera_node.py`: Records video by concatenating multiple Realsense camera sources.
- `scripts/ObjectDetection.py`: Runs the object detection model on camera frames.
- `scripts/mission_utils.py`: Utility functions for mission planning and execution.
- `scripts/tcp_server.py`: TCP server for communication with the mobile app.
- `scripts/video_stream_node.py`: Streams video feed to remote clients.
- `scripts/start_cameras.sh`: Shell script to start camera nodes.
- `scripts/start_telemetry_fowarding.sh`: Shell script to start telemetry forwarding.

### Notes

- `offboard_node.py` utilizes DroneKit for high level control. See DroneKit [documentation](https://github.com/dronekit/dronekit-python).
- The TensorFlow Lite model file `efficientdet-lite-plastic_bag2_edgetpu.tflite` is included in the `scripts/` directory for plastic bag detection. If you wish to use a different model, replace this file.
