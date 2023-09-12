import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Initialize the RealSense pipeline
pipeline1 = rs.pipeline()
pipeline2 = rs.pipeline()

# Create config objects for each camera
config1 = rs.config()
config2 = rs.config()

# Enable color stream for each camera
config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config1.enable_device("819112073352")
config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config2.enable_device("918512072489")

# Start the pipeline for each camera
profile1 = pipeline1.start(config1)
profile2 = pipeline2.start(config2)

try:
    while True:
        # Wait for the frames from both cameras
        front_frames = pipeline1.wait_for_frames(timeout_ms=5000)
        down_frames = pipeline2.wait_for_frames(timeout_ms=5000)

        # Get the color frames from each camera
        front_color = front_frames.get_color_frame()
        down_color = down_frames.get_color_frame()

        if not front_color or not down_color:
            continue

        # Convert the frames to numpy arrays
        front_color = np.asanyarray(front_color.get_data())
        down_color = np.asanyarray(down_color.get_data())

        concat = np.concatenate([front_color, down_color], axis=1)

        # Display the images from both cameras
        cv2.imshow("Camera", concat)
        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline and close all windows
    pipeline1.stop()
    pipeline2.stop()
    cv2.destroyAllWindows()