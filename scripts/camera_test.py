#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def split_video_and_publish(video_path, pub):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        rospy.logerr("Error: Unable to open video file.")
        return

    bridge = CvBridge()
    rate = rospy.Rate(100)  # Adjust the publishing rate as needed

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        height, width, _ = frame.shape
        half_height = height // 2
        half_width = width // 2

        # Splitting the frame into four quadrants
        quadrants = [
            frame[:half_height, :half_width],
            frame[:half_height, half_width:],
            frame[half_height:, :half_width],
            frame[half_height:, half_width:]
        ]

        vid_gray = cv2.cvtColor(quadrants[2], cv2.COLOR_BGR2GRAY)
        
        vid_gray = vid_gray * 8000
        image_msg = bridge.cv2_to_imgmsg(vid_gray, encoding="16UC1")
        pub.publish(image_msg)

        rate.sleep()

    cap.release()

if __name__ == "__main__":
    rospy.init_node('video_quadrant_publisher')
    video_path = "recording2024-02-12_14:28:33.585943.mp4"  # Update with your video file path
    topic_name = "front/depth/image_rect_raw"
    pub = rospy.Publisher(topic_name, Image, queue_size=10)
    try:
        split_video_and_publish(video_path, pub)
    except rospy.ROSInterruptException:
        pass
