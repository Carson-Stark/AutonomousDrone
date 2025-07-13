#!/usr/bin/env python3

######################################################
##  librealsense D4xx to MAVLink                    ##
######################################################
# Requirements: 
#   x86 based Companion Computer (for compatibility with Intel),
#   Ubuntu 18.04 (otherwise, the following installation instruction might not work),
#   Python3 (default with Ubuntu 18.04)
# Install required packages: 
#   pip3 install pyrealsense2
#   pip3 install transformations
#   pip3 install pymavlink
#   pip3 install apscheduler
#   pip3 install pyserial
#   pip3 install numba           # Only necessary if you want to optimize the performance. Require pip3 version >= 19 and llvmlite: pip3 install llvmlite==0.34.0
#   pip3 install opencv-python
#   sudo apt -y install python3-gst-1.0 gir1.2-gst-rtsp-server-1.0 gstreamer1.0-plugins-base gstreamer1.0-plugins-ugly libx264-dev
# Only necessary if you installed the minimal version of Ubuntu:
#   sudo apt install python3-opencv

# Set the path for pyrealsense2.[].so
# Otherwise, place the pyrealsense2.[].so file under the same directory as this script or modify PYTHONPATH
import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
import numpy as np
import math as m
import signal
import sys
import time
import argparse
import threading
import json
from time import sleep
from apscheduler.schedulers.background import BackgroundScheduler
import pymavlink.mavutil as mavutil
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import rospy
# from numba import njit

# In order to import cv2 under python3 when you also have ROS Kinetic installed
if os.path.exists("/opt/ros/kinetic/lib/python2.7/dist-packages"):
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 
if os.path.exists("~/anaconda3/lib/python3.7/site-packages"):
    sys.path.append('~/anaconda3/lib/python3.7/site-packages')
import cv2

# To setup video streaming
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

# To obtain ip address
import socket

######################################################
##  Depth parameters - reconfigurable               ##
######################################################

# Sensor-specific parameter, for D435: https://www.intelrealsense.com/depth-camera-d435/
DEPTH_WIDTH  = 640               # Defines the number of columns for each frame or zero for auto resolve
DEPTH_HEIGHT = 480               # Defines the number of lines for each frame or zero for auto resolve
COLOR_WIDTH  = 640
COLOR_HEIGHT = 480
FPS          = 30
DEPTH_RANGE_M = [0.2, 5.0]       # Replace with your sensor's specifics, in meter

obstacle_line_height_ratio = 0.45  # [0-1]: 0-Top, 1-Bottom. The height of the horizontal line to find distance to obstacle.
obstacle_line_thickness_pixel = 10 # [1-DEPTH_HEIGHT]: Number of pixel rows to use to generate the obstacle distance message. For each column, the scan will return the minimum value for those pixels centered vertically in the image.

USE_PRESET_FILE = True
PRESET_FILE  = "../cfg/d4xx-default.json"

RTSP_STREAMING_ENABLE = False
RTSP_PORT = "8554"
RTSP_MOUNT_POINT = "/d4xx"

# List of filters to be applied, in this order.
# https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md

#
# The filters can be tuned with opencv_depth_filtering.py script, and save the default values to here
# Individual filters have different options so one have to apply the values accordingly
#

# decimation_magnitude = 8
# filters[0][2].set_option(rs.option.filter_magnitude, decimation_magnitude)


######################################################
##  ArduPilot-related parameters - reconfigurable   ##
######################################################

# Default configurations for connection to the FCU
connection_string_default = '127.0.0.1:14541'
connection_baudrate_default = 921600

# Use this to rotate all processed data
camera_facing_angle_degree = 0

# Enable/disable each message/function individually
enable_msg_obstacle_distance = True
enable_msg_distance_sensor = False
obstacle_distance_msg_hz_default = 15.0

# lock for thread synchronization
lock = threading.Lock()

mavlink_thread_should_exit = False

debug_enable_default = 0

# default exit code is failure - a graceful termination with a
# terminate signal is possible.
exit_code = 1

######################################################
##  Global variables                                ##
######################################################

# Camera-related variables
pipe = None
depth_scale = 0.001
fx = 0
fy = 0
depth_hfov_deg = None
depth_vfov_deg = None

# The name of the display window
display_name  = 'Input/output depth'
rtsp_streaming_img = None

# Data variables
vehicle_pitch_rad = None
data = None
current_time_us = 0
last_obstacle_distance_sent_ms = 0  # value of current_time_us when obstacle_distance last sent
last_time = 0

# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# See here: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
min_depth_cm = int(DEPTH_RANGE_M[0] * 100)  # In cm
max_depth_cm = int(DEPTH_RANGE_M[1] * 100)  # In cm, should be a little conservative
distances_array_length = 72
angle_offset = None
increment_f  = None
distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_depth_cm + 1)

######################################################
##  Parsing user' inputs                            ##
######################################################

def progress(string):
    #print(string, file=sys.stdout)
    #sys.stdout.flush()
    pass

# Using default values if no specified inputs

connection_string = connection_string_default
connection_baudrate = connection_baudrate_default
obstacle_distance_msg_hz = obstacle_distance_msg_hz_default
debug_enable = debug_enable_default

if debug_enable == 1:
    progress("INFO: Debugging option enabled")
    cv2.namedWindow(display_name, cv2.WINDOW_AUTOSIZE)
else:
    progress("INFO: Debugging option DISABLED")

######################################################
##  Functions - MAVLink                             ##
######################################################

def mavlink_loop(conn, callbacks):
    '''a main routine for a thread; reads data from a mavlink connection,
    calling callbacks based on message type received.
    '''
    interesting_messages = list(callbacks.keys())
    while not mavlink_thread_should_exit:
        # send a heartbeat msg
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                0,
                                0,
                                0)
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None:
            continue
        callbacks[m.get_type()](m)

# https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
def send_obstacle_distance_message():
    global current_time_us, distances, camera_facing_angle_degree
    global last_obstacle_distance_sent_ms
    if current_time_us == last_obstacle_distance_sent_ms:
        # no new frame
        return
    last_obstacle_distance_sent_ms = current_time_us
    if angle_offset is None or increment_f is None:
        progress("Please call set_obstacle_distance_params before continue")
    else:
        conn.mav.obstacle_distance_send(
            current_time_us,    # us Timestamp (UNIX time or time since system boot)
            2,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
            distances,          # distances,    uint16_t[72],   cm
            0,                  # increment,    uint8_t,        deg
            min_depth_cm,	    # min_distance, uint16_t,       cm
            max_depth_cm,       # max_distance, uint16_t,       cm
            increment_f,	    # increment_f,  float,          deg
            angle_offset,       # angle_offset, float,          deg
            12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
        )

# https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
def send_single_distance_sensor_msg(distance, orientation):
    # Average out a portion of the centermost part
    conn.mav.distance_sensor_send(
        0,                  # ms Timestamp (UNIX time or time since system boot) (ignored)
        min_depth_cm,       # min_distance, uint16_t, cm
        max_depth_cm,       # min_distance, uint16_t, cm
        distance,           # current_distance,	uint16_t, cm	
        0,	                # type : 0 (ignored)
        0,                  # id : 0 (ignored)
        orientation,        # orientation
        0                   # covariance : 0 (ignored)
    )

# https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
def send_distance_sensor_message():
    global distances
    # Average out a portion of the centermost part
    curr_dist = int(np.mean(distances[33:38]))
    conn.mav.distance_sensor_send(
        0,# ms Timestamp (UNIX time or time since system boot) (ignored)
        min_depth_cm,   # min_distance, uint16_t, cm
        max_depth_cm,   # min_distance, uint16_t, cm
        curr_dist,      # current_distance,	uint16_t, cm	
        0,	            # type : 0 (ignored)
        0,              # id : 0 (ignored)
        int(camera_facing_angle_degree / 45),              # orientation
        0               # covariance : 0 (ignored)
    )

def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'D4xx: ' + text_to_be_sent
    #conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)

# Request a timesync update from the flight controller, for future work.
# TODO: Inspect the usage of timesync_update 
def update_timesync(ts=0, tc=0):
    if ts == 0:
        ts = int(round(time.time() * 1000))
    conn.mav.timesync_send(tc, ts)

# Listen to ATTITUDE data: https://mavlink.io/en/messages/common.html#ATTITUDE
def att_msg_callback(value):
    global vehicle_pitch_rad
    vehicle_pitch_rad = value.pitch
    if debug_enable == 1:
        progress("INFO: Received ATTITUDE msg, current pitch is %.2f degrees" % (m.degrees(vehicle_pitch_rad),))

vehicle_alt = 0.5

# Listen to AHRS2 data: https://mavlink.io/en/messages/ardupilotmega.html#AHRS2
def rng_msg_callback(value):
    global vehicle_alt
    vehicle_alt = value.distance


######################################################
##  Functions - D4xx cameras                        ##
######################################################

DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07", "0B3A", "0B5C"]


# Setting parameters for the OBSTACLE_DISTANCE message based on actual camera's intrinsics and user-defined params
def set_obstacle_distance_params():
    global angle_offset, camera_facing_angle_degree, increment_f, depth_scale, depth_hfov_deg, depth_vfov_deg, obstacle_line_height_ratio, obstacle_line_thickness_pixel
    
    # For forward facing camera with a horizontal wide view:
    #   HFOV=2*atan[w/(2.fx)],
    #   VFOV=2*atan[h/(2.fy)],
    #   DFOV=2*atan(Diag/2*f),
    #   Diag=sqrt(w^2 + h^2)
    depth_hfov_deg = m.degrees(2 * m.atan(DEPTH_WIDTH / (2 * fx)))
    depth_vfov_deg = m.degrees(2 * m.atan(DEPTH_HEIGHT / (2 * fy)))
    progress("INFO: Depth camera HFOV: %0.2f degrees" % depth_hfov_deg)
    progress("INFO: Depth camera VFOV: %0.2f degrees" % depth_vfov_deg)

    angle_offset = camera_facing_angle_degree - (depth_hfov_deg / 2)
    increment_f = depth_hfov_deg / distances_array_length
    progress("INFO: OBSTACLE_DISTANCE angle_offset: %0.3f" % angle_offset)
    progress("INFO: OBSTACLE_DISTANCE increment_f: %0.3f" % increment_f)
    progress("INFO: OBSTACLE_DISTANCE coverage: from %0.3f to %0.3f degrees" %
        (angle_offset, angle_offset + increment_f * distances_array_length))

    # Sanity check for depth configuration
    if obstacle_line_height_ratio < 0 or obstacle_line_height_ratio > 1:
        progress("Please make sure the horizontal position is within [0-1]: %s"  % obstacle_line_height_ratio)
        sys.exit()

    if obstacle_line_thickness_pixel < 1 or obstacle_line_thickness_pixel > DEPTH_HEIGHT:
        progress("Please make sure the thickness is within [0-DEPTH_HEIGHT]: %s" % obstacle_line_thickness_pixel)
        sys.exit()

# Find the height of the horizontal line to calculate the obstacle distances
#   - Basis: depth camera's vertical FOV, user's input
#   - Compensation: vehicle's current pitch angle
def find_obstacle_line_height():
    global vehicle_pitch_rad, depth_vfov_deg, DEPTH_HEIGHT

    # Basic position
    obstacle_line_height = DEPTH_HEIGHT * obstacle_line_height_ratio

    # Compensate for the vehicle's pitch angle if data is available
    if vehicle_pitch_rad is not None and depth_vfov_deg is not None:
        delta_height = m.sin(vehicle_pitch_rad / 2) / m.sin(m.radians(depth_vfov_deg) / 2) * DEPTH_HEIGHT
        obstacle_line_height += delta_height

    # Sanity check
    if obstacle_line_height < 50:
        obstacle_line_height = 50
    elif obstacle_line_height > DEPTH_HEIGHT - 50:
        obstacle_line_height = DEPTH_HEIGHT - 50
    
    return obstacle_line_height

# Calculate the distances array by dividing the FOV (horizontal) into $distances_array_length rays,
# then pick out the depth value at the pixel corresponding to each ray. Based on the definition of
# the MAVLink messages, the invalid distance value (below MIN/above MAX) will be replaced with MAX+1.
#    
# [0]    [35]   [71]    <- Output: distances[72]
#  |      |      |      <- step = width / 72
#  ---------------      <- horizontal line, or height/2
#  \      |      /
#   \     |     /
#    \    |    /
#     \   |   /
#      \  |  /
#       \ | /           
#       Camera          <- Input: depth_mat, obtained from depth image
#
# Note that we assume the input depth_mat is already processed by at least hole-filling filter.
# Otherwise, the output array might not be stable from frame to frame.
# @njit   # Uncomment to optimize for performance. This uses numba which requires llmvlite (see instruction at the top)
def distances_from_depth_image(obstacle_line_height, depth_mat, distances, min_depth_m, max_depth_m, obstacle_line_thickness_pixel):
    # Parameters for depth image
    depth_img_width  = depth_mat.shape[1]
    depth_img_height = depth_mat.shape[0]

    # Parameters for obstacle distance message
    step = depth_img_width / distances_array_length

    min_dist = 65535
    for i in range(distances_array_length):
        # Each range (left to right) is found from a set of rows within a column
        #  [ ] -> ignored
        #  [x] -> center + obstacle_line_thickness_pixel / 2
        #  [x] -> center = obstacle_line_height (moving up and down according to the vehicle's pitch angle)
        #  [x] -> center - obstacle_line_thickness_pixel / 2
        #  [ ] -> ignored
        #   ^ One of [distances_array_length] number of columns, from left to right in the image
        center_pixel = obstacle_line_height
        upper_pixel = center_pixel + 50
        lower_pixel = center_pixel - 50

        # Sanity checks
        if upper_pixel > depth_img_height - 10:
            upper_pixel = depth_img_height - 10
        elif upper_pixel < 10:
            upper_pixel = 10
        if lower_pixel > depth_img_height - 10:
            lower_pixel = depth_img_height - 10
        elif lower_pixel < 0:
            lower_pixel = 0

        # Converting depth from uint16_t unit to metric unit. depth_scale is usually 1mm following ROS convention.
        
        top = depth_mat[int(lower_pixel), int(i * step)] * depth_scale
        middle = depth_mat[int((lower_pixel + upper_pixel) / 2), int(i * step)] * depth_scale
        bottom = depth_mat[int(upper_pixel), int(i * step)] * depth_scale
        dist_m = max([top, middle, bottom])
        if top == 0 or middle == 0 or bottom == 0:
            dist_m = 0
        #min_point_in_scan = np.min(depth_mat[int(lower_pixel):int(upper_pixel), int(i * step)])
        #dist_m = min_point_in_scan * depth_scale


        # Default value, unless overwritten: 
        #   A value of max_distance + 1 (cm) means no obstacle is present. 
        #   A value of UINT16_MAX (65535) for unknown/not used.
        distances[i] = 65535
        #print(dist_m)

        # Note that dist_m is in meter, while distances[] is in cm.
        if dist_m > min_depth_m and dist_m < max_depth_m:
            distances[i] = dist_m * 100

        if dist_m < min_dist and dist_m > min_depth_m and dist_m < max_depth_m:
            min_dist = dist_m


######################################################
##  Functions - RTSP Streaming                      ##
##  Adapted from https://github.com/VimDrones/realsense-helper/blob/master/fisheye_stream_to_rtsp.py, credit to: @Huibean (GitHub)
######################################################

class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(SensorFactory, self).__init__(**properties)
        self.number_frames = 0
        self.fps = FPS
        self.duration = 1 / self.fps * Gst.SECOND
        self.launch_string = 'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ' \
                             'caps=video/x-raw,format=BGR,width={},height={},framerate={}/1 ' \
                             '! videoconvert ! video/x-raw,format=I420 ' \
                             '! x264enc speed-preset=ultrafast tune=zerolatency ' \
                             '! rtph264pay config-interval=1 name=pay0 pt=96'.format(COLOR_WIDTH, COLOR_HEIGHT, self.fps)

    def on_need_data(self, src, length):
        global rtsp_streaming_img
        frame = rtsp_streaming_img
        if frame is not None:
            data = frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.duration = self.duration
            timestamp = self.number_frames * self.duration
            buf.pts = buf.dts = int(timestamp)
            buf.offset = timestamp
            self.number_frames += 1
            retval = src.emit('push-buffer', buf)
            if retval != Gst.FlowReturn.OK:
                progress(retval)

    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)

    def do_configure(self, rtsp_media):
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name('source')
        appsrc.connect('need-data', self.on_need_data)


class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        factory = SensorFactory()
        factory.set_shared(True)
        self.get_mount_points().add_factory(RTSP_MOUNT_POINT, factory)
        self.attach(None)

def get_local_ip():
    local_ip_address = "172.25.20.179"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 1))  # connect() for UDP doesn't send packets
        local_ip_address = s.getsockname()[0]
    except:
        local_ip_address = socket.gethostbyname(socket.gethostname())
    return "172.25.20.179"


######################################################
##  Main code starts here                           ##
######################################################

def get_intrinsics(intrinsics_msg):
    global fx, fy
    if fx == 0 or fy == 0:
        fx = intrinsics_msg.K[0]
        fy = intrinsics_msg.K[4]
        set_obstacle_distance_params()

color_frame = None
def get_color(color_msg):
    global color_frame
    color_frame = bridge.imgmsg_to_cv2 (color_msg)

def get_depth(depth_msg):
    global last_time
    global current_time_us

    depth_frame = bridge.imgmsg_to_cv2 (depth_msg)

    # Store the timestamp for MAVLink messages
    current_time_us = int(round(time.time() * 1000000))

    # Extract depth in matrix form
    depth_mat = np.asanyarray(depth_frame)

    # Create obstacle distance data from depth image
    obstacle_line_height = find_obstacle_line_height()
    distances_from_depth_image(obstacle_line_height, depth_mat, distances, DEPTH_RANGE_M[0], DEPTH_RANGE_M[1], obstacle_line_thickness_pixel)

    if debug_enable == 1:
        # Prepare the data
        display_image = color_frame

        # Draw a horizontal line to visualize the obstacles' line
        x1, y1 = int(0), int(obstacle_line_height)
        x2, y2 = int(DEPTH_WIDTH), int(obstacle_line_height)
        line_thickness = obstacle_line_thickness_pixel
        cv2.line(display_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=line_thickness)
        cv2.line(display_image, (x1, y1 + 50), (x2, y2 + 50), (0, 255, 0), thickness=line_thickness)
        cv2.line(display_image, (x1, y1 - 50), (x2, y2 - 50), (0, 255, 0), thickness=line_thickness)

        # Put the fps in the corner of the image
        processing_speed = 1 / (time.time() - last_time)
        text = ("%0.2f" % (processing_speed,)) + ' fps'
        textsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
        cv2.putText(display_image,
                    text,
                    org = (int((display_image.shape[1] - textsize[0]/2)), int((textsize[1])/2)),
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale = 0.5,
                    thickness = 1,
                    color = (255, 255, 255))

        # Show the images
        cv2.imshow(display_name, display_image)
        cv2.waitKey(1)

        # Print all the distances in a line
        #progress("%s" % (str(distances)))
        
        last_time = time.time()
    return

def start_node():

    progress("INFO: Starting Vehicle communications")
    global conn
    conn = mavutil.mavlink_connection(
        connection_string,
        autoreconnect = True,
        source_system = 1,
        source_component = 93,
        baud=connection_baudrate,
        force_connected=True,
    )
    mavlink_callbacks = {
        'ATTITUDE': att_msg_callback,
        'RANGEFINDER': rng_msg_callback
    }
    mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
    mavlink_thread.start()

    conn.wait_heartbeat()

    # connecting and configuring the camera is a little hit-and-miss.
    # Start a timer and rely on a restart of the script to get it working.
    # Configuring the camera appears to block all threads, so we can't do
    # this internally.

    send_msg_to_gcs('Connecting to camera...')

    # Send MAVlink messages in the background at pre-determined frequencies
    sched = BackgroundScheduler()

    if enable_msg_obstacle_distance:
        sched.add_job(send_obstacle_distance_message, 'interval', seconds = 1/obstacle_distance_msg_hz)
        send_msg_to_gcs('Sending obstacle distance messages to FCU')
    elif enable_msg_distance_sensor:
        sched.add_job(send_distance_sensor_message, 'interval', seconds = 1/obstacle_distance_msg_hz)
        send_msg_to_gcs('Sending distance sensor messages to FCU')

    glib_loop = None
    if RTSP_STREAMING_ENABLE is True:
        send_msg_to_gcs('RTSP at rtsp://' + get_local_ip() + ':' + RTSP_PORT + RTSP_MOUNT_POINT)
        Gst.init(None)
        server = GstServer()
        glib_loop = GLib.MainLoop()
        glib_thread = threading.Thread(target=glib_loop.run, args=())
        glib_thread.start()
    else:
        send_msg_to_gcs('RTSP not streaming')

    sched.start()

    # gracefully terminate the script if an interrupt signal (e.g. ctrl-c)
    # is received.  This is considered to be abnormal termination.
    main_loop_should_quit = False
    def sigint_handler(sig, frame):
        global main_loop_should_quit
        main_loop_should_quit = True
    signal.signal(signal.SIGINT, sigint_handler)

    # gracefully terminate the script if a terminate signal is received
    # (e.g. kill -TERM).  
    def sigterm_handler(sig, frame):
        global main_loop_should_quit
        main_loop_should_quit = True
        global exit_code
        exit_code = 0

    signal.signal(signal.SIGTERM, sigterm_handler)

    global bridge
    front_sub = rospy.Subscriber('/front/depth/image_rect_raw', Image, queue_size=1, callback=get_depth)
    front_color_sub = rospy.Subscriber('/front/color/image_raw', Image, queue_size=1, callback=get_color)
    rospy.Subscriber('front/depth/camera_info', CameraInfo, callback=get_intrinsics)
    bridge = CvBridge()

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('camera_node', anonymous=True)
    rospy.loginfo("camera node started")
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass 
