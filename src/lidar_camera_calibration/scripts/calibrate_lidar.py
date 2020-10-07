#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Modified: hsh (sunskyhsh@hotmail.com)
Version : 1.2.1
Date    : Sep 7, 2020

Description:
Script to find the transformation between the ground plane and LiDAR pointcloud

Example Usage:
To perform calibration using the GUI to pick correspondences:

    $ rosrun lidar_camera_calibration calibrate_lidar.py --calibrate --pointcloud_topic /lidar_cloud_origin

    The calibrate extrinsic are saved as following:
    - PKG_PATH/calibration_data/lidar_calibration/${lidar_name}_extrinsics.npz
    - PKG_PATH/calibration_data/lidar_calibration/${lidar_name}_extrinsics.yaml
    --> 'R'     : rotation matrix (3, 3)
    --> 'T'     : translation offsets (3, )
    --> 'euler' : euler angles (3, )

'''

# Python 2/3 compatibility
from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

# Built-in modules
import os
import sys
import threading
import six

# External modules
import numpy as np
import yaml
import argparse
from sklearn.decomposition import PCA

# ROS modules
PKG = 'lidar_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy
# import tf2_ros
import ros_numpy
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import pptk
import scipy.linalg
import pcl

# local modules
from checkerboard import detect_checkerboard
from utils import *

# Global variables
PAUSE = False
FIRST_TIME = True
KEY_LOCK = threading.Lock()
TF_BUFFER = None
TF_LISTENER = None
TRANSFORM_MODE = False
# camera_position = None
# camera_look_at_position = None
# viewer = None

# Global paths
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CALIB_PATH = 'calibration_data/lidar_calibration'

'''
Keyboard handler thread
Inputs: None
Outputs: None
'''
def handle_keyboard():
    global KEY_LOCK, PAUSE
    key = six.moves.input('Press [ENTER] to pause and pick points\n')
    with KEY_LOCK: PAUSE = True

'''
Start the keyboard handler thread
Inputs: None
Outputs: None
'''
def start_keyboard_handler():
    keyboard_t = threading.Thread(target=handle_keyboard)
    keyboard_t.daemon = True
    keyboard_t.start()

'''
Calibrate the LiDAR using ground plane fitting.

Inputs:
    points3D - [numpy array] - (N, 3) array of 3D points

Outputs:
    Extrinsics saved in PKG_PATH/CALIB_PATH/extrinsics.npz
'''
def calibrate(points3D=None):
    # Load corresponding points
    folder = os.path.join(PKG_PATH, CALIB_PATH)
    if points3D is None and os.path.exists(os.path.join(folder, 'lidar_points.txt')):
        points3D = np.loadtxt(os.path.join(folder, 'lidar_points.txt'))
    
    # print("points3D:", points3D.shape)

    # Estimate extrinsics
    # best-fit linear plane

    # method 1: PCA method
    # pca = PCA()
    # pca.fit(points3D[:,:3])
    # normal_vector = pca.components_[2]

    # method 2: lstsq method
    # A = np.c_[points3D[:,0], points3D[:,1], np.ones(points3D.shape[0])]
    # C,_,_,_ = scipy.linalg.lstsq(A, points3D[:,2])    # coefficients
    # # evaluate it on grid
    # # Z = C[0]*X + C[1]*Y + C[2]
    # # or expressed using matrix/vector product
    # #Z = np.dot(np.c_[XX, YY, np.ones(XX.shape)], C).reshape(X.shape)
    # # normal vector
    # # print("C:", C)
    # normal_vector = np.array([C[0], C[1], -1])
    # normal_vector = np.array(normal_vector) / np.linalg.norm(normal_vector)
    # translation_vector = np.array([[0],[0],[C[2]]]) # shpae: (3,1)
    
    # method 3: PCL RANSAC fit plane
    pcl_pointcloud = pcl.PointCloud(points3D[:,:3].astype(np.float32))
    plane_model = pcl.SampleConsensusModelPlane(pcl_pointcloud)
    pcl_RANSAC = pcl.RandomSampleConsensus(plane_model)
    pcl_RANSAC.set_DistanceThreshold(0.03)
    pcl_RANSAC.computeModel()
    inliers = pcl_RANSAC.get_Inliers()
    points3D = points3D[inliers,:3]
    pca = PCA()
    pca.fit(points3D)
    normal_vector = pca.components_[2]
    translation_vector = np.zeros((3,1))

    if (normal_vector[2] < 0): normal_vector = -normal_vector
    # print("normal vector:", normal_vector)
    # rotation_angle = np.arccos(normal_vector[2])
    # rotation_vector_temp = np.cross(normal_vector, np.array([0,0,1]))
    # rotation_vector = rotation_angle * rotation_vector_temp / np.linalg.norm(rotation_vector_temp)

    # Convert rotation vector
    rotation_matrix = rotation_matrix_from_vectors(normal_vector, np.array([0,0,1]))
    euler = euler_from_matrix(rotation_matrix)
    # use reverse transform because of issues in the BIT-IVRC sensor_driver package
    reverse_euler_angles = np.zeros((1,3))
    reverse_rotation_matrix = rotation_matrix_from_vectors(np.array([0,0,1]), normal_vector)
    reverse_euler_angles = euler_from_matrix(reverse_rotation_matrix)
    alfa, beta, gama = reverse_euler_angles
    alfa = -alfa
    # x_offset, y_offset, z_offset = C
    transform_matrix_calibration = np.zeros((3,3))
    transform_matrix_calibration[0,0] = np.cos(beta)*np.cos(gama) - np.sin(beta)*np.sin(alfa)*np.sin(gama)
    transform_matrix_calibration[1,0] = np.cos(beta)*np.sin(gama) + np.sin(beta)*np.sin(alfa)*np.cos(gama)
    transform_matrix_calibration[2,0] = -np.cos(alfa)*np.sin(beta)

    transform_matrix_calibration[0,1] = -np.cos(alfa)*np.sin(gama)
    transform_matrix_calibration[1,1] = np.cos(alfa)*np.cos(gama)
    transform_matrix_calibration[2,1] = np.sin(alfa)

    transform_matrix_calibration[0,2] = np.sin(beta)*np.cos(gama) + np.cos(beta)*np.sin(alfa)*np.sin(gama)
    transform_matrix_calibration[1,2] = np.sin(beta)*np.sin(gama) - np.cos(beta)*np.sin(alfa)*np.cos(gama)
    transform_matrix_calibration[2,2] = np.cos(alfa)*np.cos(beta)

    # points3D = np.matmul(points3D[:,:3], transform_matrix_calibration)
    points3D = transform_matrix_calibration.dot(points3D[:,:3].T).T
    z_mean = np.mean(points3D[:,2])
    # print("Z statitics:", z_mean, np.std(points3D[:,2]))
    # viewer = pptk.viewer(points3D[:,:3])
    # viewer.wait()
    translation_vector[2] = -z_mean
    rospy.loginfo("The following values are used for tf static_transform_publisher")
    # print("rotation_matrix * normal_vector:", np.dot(rotation_matrix, normal_vector))
    # print("rotation_matrix * [0,0,1]:", np.dot(rotation_matrix, np.array([0,0,1])))
    # Display results
    print('Euler angles (RPY):', euler)
    print('Rotation Matrix:', rotation_matrix)
    print('Translation Offsets:', translation_vector)
    
    # Save extrinsics
    if (not os.path.exists(folder)):
        os.makedirs(folder)
    np.savez(os.path.join(folder, args.lidar_name + '_extrinsics.npz'),
        euler=euler, R=rotation_matrix, T=translation_vector)
    # with open(os.path.join(folder, args.lidar_name + '_extrinsics.yaml'),'w') as f:
    #     yaml.dump({'euler':list(euler), 'R':rotation_matrix.tolist(), 'T':translation_vector.tolist()},f)

    calibration_string = lidarCalibrationYamlBuf(rotation_matrix, translation_vector, euler, name=args.lidar_name)
    if (not os.path.exists(folder)):
        os.mkdir(folder)
    with open(os.path.join(folder, args.lidar_name + '_extrinsics.yaml'), 'w') as f:
        f.write(calibration_string)

    reverse_euler_angles = np.array(reverse_euler_angles)
    reverse_euler_angles *= 180./np.pi
    rospy.loginfo("Calibration finished. Copy the following offsets and angles to the lua file in sensor_driver package.")
    rospy.loginfo("x_offset: %f, y_offset: %f, z_offset: %f" % (translation_vector[0], translation_vector[1], translation_vector[2]))
    rospy.loginfo("x_angle: %f, y_angle: %f, z_angle: %f" % (-reverse_euler_angles[0], reverse_euler_angles[1], reverse_euler_angles[2]))
    return reverse_rotation_matrix, transform_matrix_calibration, translation_vector

def lidar_callback(lidar_points, lidar_pub=None):
    global FIRST_TIME, PAUSE, TF_BUFFER, TF_LISTENER, TRANSFORM_MODE
    if FIRST_TIME:
        FIRST_TIME = False
        # TF listener
        # rospy.loginfo('Setting up static transform listener')
        # TF_BUFFER = tf2_ros.Buffer()
        # TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)
        # Start keyboard handler thread
        if not TRANSFORM_MODE: start_keyboard_handler()

    elif TRANSFORM_MODE and (lidar_pub is not None):
        # points3D = ros_numpy.point_cloud2.pointcloud2_to_array(lidar_points)
        # global camera_position, camera_look_at_position, viewer
        # if (camera_position is not None):
        #     viewer.set(lookat=camera_look_at_position)
        # points3D = np.asarray(points3D.tolist())
        # if (viewer is None):
        #     viewer = pptk.viewer(points3D[:,:3])
        # else:
        #     viewer.clear()
        #     viewer.load(points3D[:,:3])
        # camera_position = viewer.get("eye")
        # camera_look_at_position = viewer.get("lookat")
        tf_transform = False
        if (tf_transform):
            # Transform the point cloud
            # try:
            #     transform = TF_BUFFER.lookup_transform('world', 'vehicle_frame', rospy.Time())
            #     # print(transform)
            #     lidar_points = do_transform_cloud(lidar_points, transform)
            # except tf2_ros.LookupException:
            #     pass
            # lidar_pub.publish(lidar_points)
            # print(lidar_points.header.frame_id) # world
            # rospy.loginfo("publish lidar_points")
            pass
        else:
            TRANSFORM_MODE = False
            # view pointcloud in pptk
            points3D = ros_numpy.point_cloud2.pointcloud2_to_array(lidar_points)
            points3D = np.asarray(points3D.tolist())
            if (len(points3D.shape)>2):
                points3D = points3D.reshape(-1, points3D.shape[-1])
            transform_matrix_calibration = np.zeros((3,3))
            folder = os.path.join(PKG_PATH, CALIB_PATH)
            extrinsics = readYAMLFile(os.path.join(folder, args.lidar_name + '_extrinsics.yaml'))
            rotation_matrix = np.array(extrinsics['R']['data']).reshape(extrinsics['R']['rows'], extrinsics['R']['cols'])
            translation_vector = np.array(extrinsics['T']['data']).reshape(extrinsics['T']['rows'], extrinsics['T']['cols'])
            euler_angles = np.array(extrinsics['euler']['data']).reshape(extrinsics['euler']['rows'], extrinsics['euler']['cols'])
            # reverse_rotation_matrix = rotation_matrix_from_vectors(np.array([0,0,1]), normal_vector)
            reverse_rotation_matrix = np.linalg.inv(rotation_matrix)
            reverse_euler_angles = euler_from_matrix(reverse_rotation_matrix)

            # C = np.array([ 0.38223287, -0.30942142, -2.26450941])
            # normal_vector = np.array([-0.34301221, 0.27764417, 0.89736076])

            # from plane normal vector to [0,0,1]
            # euler_angles = np.array([0.30005894148554135, 0.35012179279654077, 0.05346092729922942])
            # from [0,0,1] to plane normal vector
            # reverse_euler_angles = np.array([-0.006717, -0.004656, 0.000016])            
            alfa, beta, gama = reverse_euler_angles
            alfa = -alfa
            transform_matrix_calibration[0,0] = np.cos(beta)*np.cos(gama) - np.sin(beta)*np.sin(alfa)*np.sin(gama)
            transform_matrix_calibration[1,0] = np.cos(beta)*np.sin(gama) + np.sin(beta)*np.sin(alfa)*np.cos(gama)
            transform_matrix_calibration[2,0] = -np.cos(alfa)*np.sin(beta)

            transform_matrix_calibration[0,1] = -np.cos(alfa)*np.sin(gama)
            transform_matrix_calibration[1,1] = np.cos(alfa)*np.cos(gama)
            transform_matrix_calibration[2,1] = np.sin(alfa)

            transform_matrix_calibration[0,2] = np.sin(beta)*np.cos(gama) + np.cos(beta)*np.sin(alfa)*np.sin(gama)
            transform_matrix_calibration[1,2] = np.sin(beta)*np.sin(gama) - np.cos(beta)*np.sin(alfa)*np.cos(gama)
            transform_matrix_calibration[2,2] = np.cos(alfa)*np.cos(beta)

            # transform_matrix_calibration[3,0] = 0
            # transform_matrix_calibration[3,1] = 0
            # transform_matrix_calibration[3,2] = 0
            # transform_matrix_calibration[3,3] = 1.0

            # x_offset, y_offset, z_offset = translation_vector
            rospy.loginfo("Use the following offsets and angles to transform the cloud.")
            rospy.loginfo("x_offset: %f, y_offset: %f, z_offset: %f" % (translation_vector[0], translation_vector[1], translation_vector[2]))
            rospy.loginfo("x_angle: %f, y_angle: %f, z_angle: %f" % (-reverse_euler_angles[0], reverse_euler_angles[1], reverse_euler_angles[2]))
            # transform_matrix_calibration[0,3] = x_offset
            # transform_matrix_calibration[1,3] = y_offset
            # transform_matrix_calibration[2,3] = z_offset

            # print("points3D:", points3D.shape)
            # points3D = np.matmul(points3D[:,:3], transform_matrix_calibration[:3,:3]) + transform_matrix_calibration[:3,3]
            points3D = (transform_matrix_calibration.dot(points3D[:,:3].T) + translation_vector.reshape(-1,1)).T
            # points3D = (np.matmul(transform_matrix_calibration, points3D[:,:4].reshape(4,-1))).reshape(-1,4)
            # print("Z statitics:", np.mean(points3D[:,2]), np.std(points3D[:,2]))
            viewer = pptk.viewer(points3D[:,:3])
            
    elif PAUSE:
        # Extract points from message
        points3D = ros_numpy.point_cloud2.pointcloud2_to_array(lidar_points)
        points3D = np.asarray(points3D.tolist())
        if (len(points3D.shape)>2):
            points3D = points3D.reshape(-1, points3D.shape[-1])
        # print("points3D: ", points3D.shape)
        # return
        points3D = points3D[~np.isnan(points3D).any(axis=1), :]
        inrange = np.where(np.abs(points3D[:, 0] < 40) &
                        (np.abs(points3D[:, 1]) < 40) &
                        (points3D[:, 2] < 5))
        points3D = points3D[inrange[0]][:,:3]
        viewer = pptk.viewer(points3D)
        viewer.set(point_size=0.02)
        rospy.loginfo("Press [Ctrl] while clicking and dragging left mouse button to select points. Press [ENTER] in the pointcloud window to confirm the selection.")
        viewer.wait()
        indices = viewer.get('selected')
        if (len(indices) < 5): 
            rospy.logwarn("Points selection invalid! Try again.")
            return
        viewer.close()
        reverse_rotation_matrix, transform_matrix_calibration, translation_vector = calibrate(points3D=points3D[indices])
        # points3D = np.matmul(points3D, rotation_matrix) + translation_vector.T
        # transform_matrix_calibration[3,0] = 0
        # transform_matrix_calibration[3,1] = 0
        # transform_matrix_calibration[3,2] = 0
        # transform_matrix_calibration[3,3] = 1.0

        # x_offset, y_offset, z_offset = translation_vector
        # transform_matrix_calibration[0,3] = x_offset
        # transform_matrix_calibration[1,3] = y_offset
        # transform_matrix_calibration[2,3] = z_offset

        # print("points3D:", points3D.shape)
        points3D = (transform_matrix_calibration.dot(points3D[:,:3].T) + translation_vector.reshape(-1,1)).T
        # points3D = np.matmul(points3D[:,:3], transform_matrix_calibration[:3,:3]) + transform_matrix_calibration[:3,3]

        # points3D = np.matmul(points3D, transform_matrix_calibration[:3,:3]) + transform_matrix_calibration[:3,3]
        try:
            viewer.clear()
            viewer.load(points3D)
            viewer.set(point_size=0.02)
        except Exception:
            # Current exception is 'exc'. For python2/3 compatibility
            # import sys
            # exc = sys.exc_info()[1]
            viewer = pptk.viewer(points3D)
            viewer.set(point_size=0.02)
        rospy.loginfo("Please check the calibrated pointcloud in the new window. You can quit now by clicking [Ctrl+C].")

        # Resume listener
        with KEY_LOCK: PAUSE = False
        # start_keyboard_handler()

'''
The main ROS node which handles the topics

Inputs:
    lidar_points - [str] - ROS lidar pointcloud topic
    output_topic - [str] - ROS transformed pointcloud topic

Outputs: None
'''
def listener(lidar_points, output_topic):
    # Start node
    rospy.init_node('calibrate_lidar', anonymous=True)
    rospy.loginfo('Current PID: [%d]' % os.getpid())
    rospy.loginfo('PointCloud2 topic: %s' % lidar_points)

    # Publish output topic
    lidar_pub = None
    if TRANSFORM_MODE:
        rospy.loginfo("Transform mode.")
        lidar_pub = rospy.Publisher(output_topic, PointCloud2, queue_size=2)
    else:
        rospy.loginfo("Calibrate mode.")

    # Subscribe to topics
    lidar_sub = rospy.Subscriber(lidar_points, PointCloud2, callback=lidar_callback, callback_args=lidar_pub)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument('--transform_mode', action="store_true")
    parser.add_argument('--pointcloud_topic', type=str, default='/lidar_cloud_calibrated')
    parser.add_argument('--lidar_name', type=str, default='HDL32')
    parser.add_argument('--output_topic', type=str, default='/lidar_cloud_calibrated')
    # parser.add_argument('__name', type=str, default='calibrate_lidar')
    # parser.add_argument('__log', type=str)
    args = parser.parse_args()

    # Calibration mode
    if args.calibrate:
        pointcloud_topic = args.pointcloud_topic
        TRANSFORM_MODE = False
        output_topic = None
    # Transform mode
    elif args.transform_mode:
        pointcloud_topic = args.pointcloud_topic
        # pointcloud_topic = rospy.get_param('pointcloud_topic')
        TRANSFORM_MODE = True #bool(rospy.get_param('transform_mode'))
        output_topic = pointcloud_topic + "_transformed"
    else:
        print("Wrong arguments! exit.")
        sys.exit()

    # Start keyboard handler thread
    # if not TRANSFORM_MODE: start_keyboard_handler()

    # Start subscriber
    listener(pointcloud_topic, args.output_topic)
