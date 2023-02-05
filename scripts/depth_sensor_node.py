#!/usr/bin/env python3
import os
import cv2
import numpy as np
import ros_numpy
import rospy
import sensor_msgs.point_cloud2 as pcl2
import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from StereoCuda import StereoCuda
from cv_bridge import CvBridge

def start_node():
    cv_file = cv2.FileStorage()
    script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
    map_file_path = os.path.join(script_dir, 'stereoMap.xml')
    print (map_file_path)
    cv_file.open(str(map_file_path), cv2.FileStorage_READ)

    global stereoMapL_x
    global stereoMapL_y
    global stereoMapR_x
    global stereoMapR_y
    stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
    stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
    stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
    stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

    left_cam = message_filters.Subscriber('leftcam', Image)
    right_cam = message_filters.Subscriber('rightcam', Image)
    ts = message_filters.ApproximateTimeSynchronizer([left_cam, right_cam], queue_size=1, slop=0.5)
    ts.registerCallback(process_frame)

    global bridge
    bridge = CvBridge()
    
    global pointcloud_pub
    pointcloud_pub = rospy.Publisher('camera/depth/points', PointCloud2, queue_size=10)
    #pointcloud_pub = rospy.Publisher('disparity', Image, queue_size=1)

    rospy.spin()

    cv2.destroyAllWindows()

def process_frame(left_img, right_img):
    front_left = bridge.imgmsg_to_cv2(left_img)
    front_right = bridge.imgmsg_to_cv2(right_img)

    point_cloud = get_front_pointcloud(front_left, front_right, False)
    pointcloud_pub.publish(point_cloud)
    

def get_front_pointcloud(front_left, front_right, show_img=False):
    block_size = 21
    min_disparity = 0
    num_disparities = 64
    uniqueness_ratio = 2
    
    scale_factor = 375

    front_right = cv2.remap(front_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, 0)
    front_left = cv2.remap(front_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, 0)
    front_right = cv2.resize (front_right, (480, 270))
    front_left = cv2.resize (front_left, (480, 270))

    disparity = get_depth_map (front_left, front_right, StereoCuda(num_disparities=num_disparities, block_size=block_size, \
        min_disparity=min_disparity, uniqueness_ratio=uniqueness_ratio))

    disparity = cv2.medianBlur (disparity, 5)

    if show_img:
        cv2.imshow ("front left", front_left)
        cv2.imshow("Disparity", cv2.applyColorMap(disparity, cv2.COLORMAP_JET))

    points = reconstruct3d(disparity, front_left)
    points = points.reshape(-1, 3)
    points = points[::2,:] #downsample by factor of 2
    points[:,2] *= -1
    translation = np.tile ([-240, 135, 3 * scale_factor], (points.shape[0], 1))
    points = (points + translation) / scale_factor
    #list(map(tuple, points))
    dtype = np.dtype([("x", np.float32), ("z", np.float32), ("y", np.float32)])
    points = np.core.records.fromarrays(points.transpose(), dtype=dtype)

    header = Header()
    header.stamp = rospy.Time.now()
    pointcloud = ros_numpy.point_cloud2.array_to_pointcloud2 (points, stamp=rospy.Time.now(), frame_id="camera_link")
    pointcloud.is_dense = False

    return pointcloud

def get_depth_map(left, right, stereo):

    #disparity = stereo.compute(grayLeft, grayRight)
    disparity = stereo.compute_disparity(left, right)

    #Normalize the values to a range from 0..255 for a grayscale image
    disparity = cv2.normalize(disparity, disparity, alpha=255,
                                beta=0, norm_type=cv2.NORM_MINMAX)
    disparity = np.uint8(disparity)
    return disparity

def reconstruct3d(disparity, orig, write_pointcloud=False):
    disparity = disparity[20:-20, 20:-20]
    orig = orig[20:-20, 20:-20]
    Q1 = np.float32([[1,0,0,0],
                    [0,-1,0,0],
                    [0,0,5,0],
                    [0,0,0,1]])
    Q2 = np.float32([[1,0,0,0],
                    [0,-1,0,0],
                    [0,0,0,1]])
    points3d = cv2.reprojectImageTo3D(disparity, Q1, handleMissingValues=0)
    mask_map = disparity > disparity.min()
    output_points = points3d[mask_map]
    if write_pointcloud:
        colors = cv2.cvtColor(orig, cv2.COLOR_BGR2RGB)
        output_colors = colors[mask_map]
        self.write_pointcloud(output_points, output_colors, "pointcloud.ply")
    
    return output_points

def write_pointcloud(vertices, colors, filename):
    colors = colors.reshape(-1, 3)
    vertices = np.hstack([vertices.reshape(-1,3), colors])

    ply_header = '''ply
        format ascii 1.0
        element vertex %(vert_num)d
        property float x
        property float y
        property float z
        property uchar red
        property uchar green
        property uchar blue
        end_header
    '''

    with open(filename, 'w') as f:
        f.write(ply_header % dict(vert_num=len(vertices)))
        np.savetxt(f, vertices, '%f %f %f %d %d %d')
    
    print (f"saved pointcloud to {filename}")

if __name__ == '__main__':
    rospy.init_node('depth_sensor_node', anonymous=True)
    rospy.loginfo("serving point cloud")
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
