#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Modified: hsh (sunskyhsh@hotmail.com)
Version : 1.2.1
Date    : Sep 7, 2020

Description:
Script to find the transformation between the Camera and the LiDAR

Example Usage:
1. To perform calibration using the GUI to pick correspondences chessboard points:

    $ rosrun lidar_camera_calibration calibrate_camera_lidar.py --calibrate \
        --image_topic XXX --pointcloud_topic XXX [--camera_lidar_topic XXX \
        --chessboard_size 7x9 --square_size 0.12 --lidar_points 500]
        
    The corresponding images and pointclouds will be save as following:
    - PKG_PATH/calibration_data/lidar_camera_calibration/[camera_name]_pcd/[0...n].jpg
    - PKG_PATH/calibration_data/lidar_camera_calibration/[camera_name]_pcd/[0...n].pcd

    The calibrate extrinsic are saved as following:
    - PKG_PATH/calibration_data/lidar_camera_calibration/extrinsics.yaml
    --> 'R'     : rotation matrix (3, 3)
    --> 'T'     : translation offsets (3, )
    --> 'euler' : euler angles (3, )

2. To display the LiDAR points projected on to the camera plane:

    $ rosrun lidar_camera_calibration calibrate_camera_lidar.py --project \
        --image_topic XXX --pointcloud_topic XXX [--camera_lidar_topic XXX]

Notes:
Make sure this file has executable permissions:
$ chmod +x calibrate_camera_lidar.py

Reference:
Huang, Lili, and Matthew Barth. "A novel multi-planar LIDAR and computer vision calibration procedure using 2D patterns for automated navigation." 2009 IEEE Intelligent Vehicles Symposium. IEEE, 2009.
'''

# Python 2/3 compatibility
from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

# Built-in modules
import os
import sys
import time
import threading
import multiprocessing
import math
import six

# External modules
import cv2
import numpy as np
import matplotlib.cm
import yaml
import argparse
from scipy import optimize

# ROS modules
PKG = 'lidar_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import rosbag
import rospy
import ros_numpy
import image_geometry
import message_filters
from cv_bridge import CvBridge, CvBridgeError
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, CompressedImage
import pptk
import pcl

# local modules
from checkerboard import detect_checkerboard
from utils import *

# Global variables
PAUSE = False
FIRST_TIME = True
CAMERA_MODEL_INITED = False
KEY_LOCK = threading.Lock()
TF_BUFFER = None
TF_LISTENER = None
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()
IMAGE_COMPRESSED = False
WATCH_LIVESTREAM_OUTCOME = False
REMOVE_LAST_FRAME = False

# Global paths
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CALIB_PATH = 'calibration_data/lidar_camera_calibration'


'''
Keyboard handler thread
Inputs: None
Outputs: None
'''
def handle_keyboard():
    global KEY_LOCK, PAUSE, WATCH_LIVESTREAM_OUTCOME, REMOVE_LAST_FRAME
    key = six.moves.input('Press [ENTER] to pause and pick points or \n[l] to watch livestream outcome or \n[c] to cancel livestream mode\n[r] to remove last frame\n')
    if (key == 'l'):
        print("[l] pressed. watch livestream outcome")
        with KEY_LOCK:
            PAUSE = True
            WATCH_LIVESTREAM_OUTCOME = True
    elif (key == 'c'):
        print("[c] pressed. cancel livestream mode")
        with KEY_LOCK:
            PAUSE = True
            WATCH_LIVESTREAM_OUTCOME = False
    elif (key == 'r'):
        print("[r] pressed. remove last frame")
        with KEY_LOCK:
            REMOVE_LAST_FRAME = False
    else:
        print("[ENTER] pressed")
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


class CameraLiDARCalibrator:
    '''
    The main ROS node which handles the topics

    Inputs:
        camera_info - [str] - ROS sensor camera info topic
        image_color - [str] - ROS sensor image topic
        velodyne - [str] - ROS velodyne PCL2 topic
        camera_lidar - [str] - ROS projected points image topic

    Outputs: None
    '''
    def __init__(self, args, image_topic, pointcloud_topic, camera_lidar_topic=None):
        self.args = args
        # Start node
        rospy.init_node('calibrate_camera_lidar', anonymous=True)
        rospy.loginfo('Current PID: [%d]' % os.getpid())
        rospy.loginfo('Projection mode: %s' % PROJECT_MODE)
        rospy.loginfo('Image topic: %s' % image_topic)
        rospy.loginfo('PointCloud2 topic: %s' % pointcloud_topic)
        rospy.loginfo('Output topic: %s' % camera_lidar_topic)
        self.calibrate_mode = self.args.calibrate
        assert ('x' in args.chessboard_size)
        self.chessboard_size = tuple([int(x) for x in args.chessboard_size.split('x')])
        assert (len(self.chessboard_size) == 2)
        self.chessboard_diagonal = np.sqrt(self.chessboard_size[0]**2+self.chessboard_size[1]**2) * args.square_size
        # chessboard refine termination criteria
        self.chessboard_corner_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.chessboard_corner_coords = np.zeros((self.chessboard_size[0]*self.chessboard_size[1],3), np.float32)
        self.chessboard_corner_coords[:,:2] = np.mgrid[0:self.chessboard_size[0],0:self.chessboard_size[1]].T.reshape(-1,2) * args.square_size
        self.camera_name = image_topic.split('/')[1]
        self.camera_matrix = None
        self.image_shape = None
        self.board_points = []
        self.pixel_points = []
        self.lidar_points = []
        self.chessboard_to_camera_rvecs = []
        self.chessboard_to_camera_tvecs = []
        self.frame_count = 0
        self.use_L1_error = args.use_L1_error
        self.calibrate_min_frames = args.calibrate_min_frames
        self.chessboard_ROI_points = []
        self.procs = []
        self._threads = []
        self._lock = threading.Lock()
        self.calibration_finished = False
        self.lidar_to_camera_R = None
        self.lidar_to_camera_T = None
        self.folder = os.path.join(PKG_PATH, CALIB_PATH)
        self.success = False
        self.board_points_num = 0
        self.pixel_points_num = 0
        self.lidar_points_num = 0
        if (not PROJECT_MODE):
            rospy.loginfo("chessboard_size: %s" % str(self.chessboard_size))
            rospy.loginfo("chessboard_diagonal: %.2f" % (self.chessboard_diagonal))
            rospy.loginfo("use_L1_error: %s" % str(self.use_L1_error))
        if (args.transform_pointcloud):
            self.lidar_rotation_matrix = np.zeros((3,3))
            extrinsics = readYAMLFile(os.path.join(PKG_PATH, 'calibration_data/lidar_calibration', self.args.lidar_name+'_extrinsics.yaml'))
            rotation_matrix = np.array(extrinsics['R']['data']).reshape(extrinsics['R']['rows'], extrinsics['R']['cols'])
            translation_vector = np.array(extrinsics['T']['data']).reshape(extrinsics['T']['rows'], extrinsics['T']['cols'])
            self.lidar_translation_vector = np.array(extrinsics['T']['data']).reshape(extrinsics['T']['rows'], extrinsics['T']['cols'])
            euler_angles = np.array(extrinsics['euler']['data']).reshape(extrinsics['euler']['rows'], extrinsics['euler']['cols'])
            # reverse_rotation_matrix = rotation_matrix_from_vectors(np.array([0,0,1]), normal_vector)
            reverse_rotation_matrix = np.linalg.inv(rotation_matrix)
            reverse_euler_angles = euler_from_matrix(reverse_rotation_matrix)

            alfa, beta, gama = reverse_euler_angles
            alfa = -alfa
            self.lidar_rotation_matrix[0,0] = np.cos(beta)*np.cos(gama) - np.sin(beta)*np.sin(alfa)*np.sin(gama)
            self.lidar_rotation_matrix[1,0] = np.cos(beta)*np.sin(gama) + np.sin(beta)*np.sin(alfa)*np.cos(gama)
            self.lidar_rotation_matrix[2,0] = -np.cos(alfa)*np.sin(beta)

            self.lidar_rotation_matrix[0,1] = -np.cos(alfa)*np.sin(gama)
            self.lidar_rotation_matrix[1,1] = np.cos(alfa)*np.cos(gama)
            self.lidar_rotation_matrix[2,1] = np.sin(alfa)

            self.lidar_rotation_matrix[0,2] = np.sin(beta)*np.cos(gama) + np.cos(beta)*np.sin(alfa)*np.sin(gama)
            self.lidar_rotation_matrix[1,2] = np.sin(beta)*np.sin(gama) - np.cos(beta)*np.sin(alfa)*np.cos(gama)
            self.lidar_rotation_matrix[2,2] = np.cos(alfa)*np.cos(beta)

            # self.lidar_rotation_matrix[3,0] = 0
            # self.lidar_rotation_matrix[3,1] = 0
            # self.lidar_rotation_matrix[3,2] = 0
            # self.lidar_rotation_matrix[3,3] = 1.0

        # Subscribe to topics
        if (IMAGE_COMPRESSED):
            image_sub = message_filters.Subscriber(image_topic, CompressedImage)
        else:
            image_sub = message_filters.Subscriber(image_topic, Image)
        pointcloud_sub = message_filters.Subscriber(pointcloud_topic, PointCloud2)

        # Publish output topic
        image_pub = None
        if camera_lidar_topic: image_pub = rospy.Publisher(camera_lidar_topic, Image, queue_size=5)

        # Synchronize the topics by time
        ats = message_filters.ApproximateTimeSynchronizer(
            [image_sub, pointcloud_sub], queue_size=5, slop=0.1)
        ats.registerCallback(self.camera_lidar_callback, image_pub)

        # Start keyboard handler thread
        if not args.project: start_keyboard_handler()

        # Keep python from exiting until this node is stopped
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo('Shutting down')

    '''
    Runs the image point selection GUI process

    Inputs:
        img_msg - [sensor_msgs/Image] - ROS sensor image message
        now - [int] - ROS bag time in seconds
        rectify - [bool] - to specify whether to rectify image or not

    Outputs:
        Picked points saved in PKG_PATH/CALIB_PATH/img_corners.npy
    '''
    def extract_points_2D(self, img_msg, now, proc_results, calibrator_instance, rectify=False):
        # print("board_points:", len(board_points), ":", [x.shape for x in board_points])
        # Log PID
        rospy.loginfo('2D Picker PID: [%d]' % os.getpid())

        # Read image using CV bridge
        try:
            if (IMAGE_COMPRESSED):
                img = CV_BRIDGE.compressed_imgmsg_to_cv2(img_msg, 'bgr8')
            else:
                img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
        except CvBridgeError as e: 
            rospy.logerr(e)
            return None
        rospy.loginfo('image msg converted.')

        # Rectify image
        if rectify and CAMERA_MODEL_INITED: CAMERA_MODEL.rectifyImage(img, img)
        cv2.imwrite(os.path.join(PKG_PATH, CALIB_PATH, self.camera_name+"_pcd", str(self.board_points_num)+".jpg"), img)

        # gamma = 2.
        # invGamma = 1.0 / gamma
        # table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
        # # apply gamma correction using the lookup table
        # img = cv2.LUT(np.array(img, dtype = np.uint8), table)
        # cv2.namedWindow('gamma_transform', 0)
        # cv2.imshow("gamma_transform", img)
        # cv2.waitKey(0)
        # detect corners
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        # corners2 = None
        if ret == True:
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.chessboard_corner_criteria)
            # Draw and display the corners
            cv2.drawChessboardCorners(img, self.chessboard_size, corners, ret)
            cv2.namedWindow('chessboard corner detect', 0)
            cv2.imshow('chessboard corner detect', img)
            key = cv2.waitKey(0)
            if (key == ord('q')):
                cv2.destroyAllWindows()
            proc_results.put([self.chessboard_corner_coords, np.squeeze(corners)])
        else:
            rospy.logwarn("No chessboard corners found in the current image! Try another chessboard corners detection method...")
            corners, score = detect_checkerboard(gray, self.chessboard_size[::-1])
            if (corners is None):
                rospy.logerr("Chessboard corner detection failed! Skip this frame.")
                return
            corners = corners.astype(np.float32)
            proc_results.put([self.chessboard_corner_coords, np.squeeze(corners)])
            cv2.drawChessboardCorners(img, self.chessboard_size, corners, True)
            cv2.namedWindow('chessboard', 0)
            cv2.imshow('chessboard', img)
            cv2.resizeWindow('chessboard', 640,480)
            cv2.moveWindow('chessboard', 500,0)
            key = cv2.waitKey(0)
            if (key==13 or key==32): #按空格和回车键退出
                # cv2.destroyAllWindows()
                pass
            cv2.destroyWindow('chessboard')


    '''
    Runs the LiDAR point selection GUI process

    Inputs:
        velodyne - [sensor_msgs/PointCloud2] - ROS velodyne PCL2 message
        now - [int] - ROS bag time in seconds

    Outputs:
        Picked points saved in PKG_PATH/CALIB_PATH/pcl_corners.npy
    '''
    def extract_points_3D(self, pointcloud_msg, now, proc_results, calibrator_instance):
        # print("lidar_points:", len(lidar_points), ":", [x.shape for x in lidar_points])
        # Log PID
        rospy.loginfo('3D Picker PID: [%d]' % os.getpid())

        # Extract points data
        points = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud_msg)
        points = np.asarray(points.tolist())
        if (len(points.shape)>2):
            points = points.reshape(-1, points.shape[-1])
        points = points[~np.isnan(points).any(axis=1), :]
        points2pcd(points, os.path.join(PKG_PATH, CALIB_PATH, self.camera_name+"_pcd", str(self.lidar_points_num)+".pcd"))
        if (self.args.transform_pointcloud):
            points = (self.lidar_rotation_matrix.dot(points[:,:3].T) + self.lidar_translation_vector.reshape(-1,1)).T

        # Select points within chessboard range
        inrange = np.where((np.abs(points[:, 0]) < 7) &
                        (points[:, 1] < 25) &
                        (points[:, 1] > 0) &
                        # (points[:, 2] < 4) &
                        (points[:, 2] > 0.2))
        points = points[inrange[0]]
        if points.shape[0] < 10:
            rospy.logwarn('Too few PCL points available in range')
            return

        # pptk viewer
        viewer = pptk.viewer(points[:, :3])
        viewer.set(lookat=(0,0,4))
        viewer.set(phi=-1.57)
        viewer.set(theta=0.4)
        viewer.set(r=4)
        viewer.set(floor_level=0)
        viewer.set(point_size=0.02)
        rospy.loginfo("Press [Ctrl + LeftMouseClick] to select a chessboard point. Press [Enter] on viewer to finish selection.")
        pcl_pointcloud = pcl.PointCloud(points[:,:3].astype(np.float32))
        region_growing = pcl_pointcloud.make_RegionGrowing(searchRadius=self.chessboard_diagonal/5.0)
        indices = []
        viewer.wait()
        indices = viewer.get('selected')
        if (len(indices) < 1):
            rospy.logwarn("No point selected!")
            # self.terminate_all()
            viewer.close()
            return
        rg_indices = region_growing.get_SegmentFromPoint(indices[0])
        points_color = np.zeros(len(points))
        points_color[rg_indices] = 1
        viewer.attributes(points_color)
        # viewer.wait()
        chessboard_pointcloud = pcl_pointcloud.extract(rg_indices)
        plane_model = pcl.SampleConsensusModelPlane(chessboard_pointcloud)
        pcl_RANSAC = pcl.RandomSampleConsensus(plane_model)
        pcl_RANSAC.set_DistanceThreshold(0.01)
        pcl_RANSAC.computeModel()
        inliers = pcl_RANSAC.get_Inliers()
        # random select a fixed number of lidar points to reduce computation time
        inliers = np.random.choice(inliers, self.args.lidar_points)
        chessboard_points = np.asarray(chessboard_pointcloud)
        proc_results.put([np.asarray(chessboard_points[inliers, :3])])
        points_color = np.zeros(len(chessboard_points))
        points_color[inliers] = 1
        viewer.load(chessboard_points[:,:3])
        viewer.set(lookat=(0,0,4))
        viewer.set(phi=-1.57)
        viewer.set(theta=0.4)
        viewer.set(r=2)
        viewer.set(floor_level=0)
        viewer.set(point_size=0.02)
        viewer.attributes(points_color)
        viewer.attributes(points_color)
        rospy.loginfo("Check the chessboard segmentation in the viewer.")
        viewer.wait()
        viewer.close()

    '''
    Calibrate the LiDAR and image points using OpenCV PnP RANSAC
    Requires minimum 5 point correspondences

    Inputs:
        points2D - [numpy array] - (N, 2) array of image points
        points3D - [numpy array] - (N, 3) array of 3D points

    Outputs:
        Extrinsics saved in PKG_PATH/CALIB_PATH/extrinsics.npz
    '''
    def calibrate(self, points2D=None, points3D=None):
        # print("lidar_points:", len(self.lidar_points), ":", [x.shape for x in self.lidar_points])
        # print("chessboard_to_camera_rvecs:", len(self.chessboard_to_camera_rvecs), ":", [x.shape for x in self.chessboard_to_camera_rvecs])
        # 1. linear calibration
        if (self.lidar_to_camera_R is None or self.lidar_to_camera_T is None):
            rospy.loginfo("Step 1: Linear calibration.")
            total_lidar_points = np.row_stack(self.lidar_points)
            mean_lidar_points = total_lidar_points.mean(axis=0)
            # print("mean_lidar_points:", mean_lidar_points.shape)
            diff_lidar_points = total_lidar_points - mean_lidar_points
            scale_lidar_points = np.abs(diff_lidar_points).max(axis=0).mean()
            # print("scale_lidar_points:", scale_lidar_points)
            A = None
            b = None
            for i in range(self.frame_count):
                lidar_points = (self.lidar_points[i] - mean_lidar_points) / scale_lidar_points
                r3 = self.chessboard_to_camera_R[i][:,2].reshape(-1,1) # last column of R (normal vector of chessboard plane in camera coordinate)
                # r3_matrix = np.zeros((3,12))
                # r3_matrix[0,0], r3_matrix[1,1], r3_matrix[2,2] = r3[0]
                # r3_matrix[]
                n_points = lidar_points.shape[0]
                if (A is None):
                    A = np.c_[r3[0]*lidar_points, r3[0]*np.ones((n_points,1)), 
                        r3[1]*lidar_points, r3[1]*np.ones((n_points,1)), 
                        r3[2]*np.ones((n_points,1)), r3[2]*lidar_points]
                    # print("A form None to", A.shape)
                    b = r3.T.dot(self.chessboard_to_camera_tvecs[i])[0] / scale_lidar_points * np.ones((n_points,1))
                else:
                    A = np.r_[A, np.c_[r3[0]*lidar_points, r3[0]*np.ones((n_points,1)), 
                        r3[1]*lidar_points, r3[1]*np.ones((n_points,1)), 
                        r3[2]*np.ones((n_points,1)), r3[2]*lidar_points]]
                    b = np.r_[b, r3.T.dot(self.chessboard_to_camera_tvecs[i])[0] / scale_lidar_points * np.ones((n_points,1))]
            print("A:", A.shape, ". b:", b.shape)
            mm = np.linalg.inv(A.T.dot(A)).dot(A.T).dot(b).squeeze()
            tR = np.array([[mm[0],mm[1],mm[2]],[mm[4],mm[5],mm[6]],[mm[9],mm[10],mm[11]]])
            uu, dd, vv = np.linalg.svd(tR)
            self.lidar_to_camera_R = uu.dot(vv.T)
            self.lidar_to_camera_T = scale_lidar_points*mm[[3,7,8]].reshape(-1,1) - self.lidar_to_camera_R.dot(mean_lidar_points.reshape(-1,1))
            print("R:", self.lidar_to_camera_R)
            print("T:", self.lidar_to_camera_T)
            # 2. Non-linear calibration
            rospy.loginfo("Step 2: Non-linear calibration.")

        rvec_and_tvec0 = np.squeeze(np.r_[cv2.Rodrigues(self.lidar_to_camera_R)[0],self.lidar_to_camera_T])
        print("initial value [rvec_and_tvec0]:", rvec_and_tvec0)
        optimize_ret = optimize.least_squares(self.calibProjErrLidar, rvec_and_tvec0) #, bounds=(-3.14,3.14))
        rvec_and_tvec = optimize_ret.x
        self.success = optimize_ret.success
        print("camera and lidar chessboard plane error:", np.mean(optimize_ret.fun))
        # print("original optimize_ret:", optimize_ret)
        print("optimized value [rvec_and_tvec]:", rvec_and_tvec)
        print("optimization success status:", self.success)
        self.lidar_to_camera_R = cv2.Rodrigues(rvec_and_tvec[0:3])[0]
        self.lidar_to_camera_T = rvec_and_tvec[3:6].reshape(-1,1)
        print("R:", self.lidar_to_camera_R)
        print("T:", self.lidar_to_camera_T)

        euler = euler_from_matrix(self.lidar_to_camera_R)
        
        # Save extrinsics
        np.savez(os.path.join(self.folder, 'extrinsics_'+self.camera_name+'_and_'+self.args.lidar_name+'.npz'),
            euler=euler, R=self.lidar_to_camera_R, T=self.lidar_to_camera_T.T)
        # writeYAMLFile(os.path.join(self.folder, 'extrinsics_'+self.camera_name+'_and_'+self.args.lidar_name+'.yaml'), {'euler':list(euler), 'R':self.lidar_to_camera_R.tolist(), 'T':self.lidar_to_camera_T.T.tolist()})

        calibration_string = cameraLidarCalibrationYamlBuf(self.lidar_to_camera_R, self.lidar_to_camera_T, self.camera_matrix)
        if (not os.path.exists(self.folder)):
            os.mkdir(self.folder)
        with open(os.path.join(self.folder, 'extrinsics_'+self.camera_name+'_and_'+self.args.lidar_name+'.yaml'), 'w') as f:
            f.write(calibration_string)

    def calibProjErrLidar(self, rvec_and_tvec):
        err = None
        rotation_matrix = np.squeeze(cv2.Rodrigues(rvec_and_tvec[0:3])[0])
        for i in range(self.frame_count):
            r3 = self.chessboard_to_camera_R[i][:,2].reshape(-1,1)
            n_points = len(self.lidar_points[i])
            diff = r3.T.dot(rotation_matrix.dot(self.lidar_points[i].T) + rvec_and_tvec[3:6].reshape(-1,1) - self.chessboard_to_camera_tvecs[i])
            if (self.use_L1_error):
                curr_err = np.abs(diff).mean()
            else:
                curr_err = np.sqrt(np.square(diff).mean())
            if (err is None):
                err = curr_err
            else:
                err = np.r_[err, curr_err]
        return err

    '''
    Projects the point cloud on to the image plane using the extrinsics

    Inputs:
        img_msg - [sensor_msgs/Image] - ROS sensor image message
        pointcloud_msg - [sensor_msgs/PointCloud2] - ROS pointcloud_msg PCL2 message
        image_pub - [sensor_msgs/Image] - ROS image publisher

    Outputs:
        Projected points published on /sensors/camera/camera_lidar topic
    '''
    def project_point_cloud(self, pointcloud_msg, img_msg, image_pub):
        # Read image using CV bridge
        try:
            if (IMAGE_COMPRESSED):
                img = CV_BRIDGE.compressed_imgmsg_to_cv2(img_msg, 'bgr8')
            else:
                img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
        except CvBridgeError as e: 
            rospy.logerr(e)
            return

        if (self.lidar_to_camera_R is None or self.lidar_to_camera_T is None):
            extrinsic_fname = 'extrinsics_'+self.camera_name+'_and_'+self.args.lidar_name+'.yaml'
            rospy.loginfo("Reading extrinsics from %s" % os.path.join(self.folder, extrinsic_fname))
            extrinsics = readYAMLFile(os.path.join(self.folder, extrinsic_fname))
            if ('RT' in extrinsics):
                self.lidar_to_camera_RT = np.array(extrinsics['RT']['data']).reshape(extrinsics['RT']['rows'], extrinsics['RT']['cols'])
                print("RT:", self.lidar_to_camera_RT.shape)
                self.lidar_to_camera_R = self.lidar_to_camera_RT[:3,:3]
                self.lidar_to_camera_T = self.lidar_to_camera_RT[:3,3].reshape(-1,1)
            else:
                self.lidar_to_camera_R = np.array(extrinsics['R'])
                self.lidar_to_camera_T = np.array(extrinsics['T'])
        if (self.camera_matrix is None):
            rospy.loginfo("Reading intrinsics from %s" % os.path.join(self.folder, "intrinsics_"+self.camera_name+".yaml"))
            intrinsics = readYAMLFile(os.path.join(self.folder, "intrinsics_"+self.camera_name+".yaml"))
            self.camera_matrix = np.array(intrinsics['camera_matrix']['data']).reshape(3,3)
        CAMERA_MODEL_INITED
        # Transform the point cloud
        # try:
        #     transform = TF_BUFFER.lookup_transform('world', 'vehicle_frame', rospy.Time())
        #     pointcloud_msg = do_transform_cloud(pointcloud_msg, transform)
        # except tf2_ros.LookupException:
        #     pass

        # Extract points from message
        points3D = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud_msg)
        points3D = np.asarray(points3D.tolist())
        if (len(points3D.shape)>2):
            points3D = points3D.reshape(-1, points3D.shape[-1])
        points3D = points3D[~np.isnan(points3D).any(axis=1), :]
        
        if (self.args.transform_pointcloud):
            points3D = (self.lidar_rotation_matrix.dot(points3D[:,:3].T) + self.lidar_translation_vector.reshape(-1,1)).T

        # Filter points in front of camera
        inrange = np.where((np.abs(points3D[:, 0]) < 8) &
                           (points3D[:, 1] > 0.2) & (points3D[:,1] < 20) & (points3D[:,2] < 6) & (points3D[:,2] > 0.1))
        points3D = points3D[inrange[0]]
        max_intensity = np.max(points3D[:, -1])
        # Color map for the points
        cmap = matplotlib.cm.get_cmap('jet')
        colors = cmap(points3D[:, -1] / max_intensity) * 255
        points3D = points3D[:,:3]
        points3D_T = (self.lidar_to_camera_R.dot(points3D.T) + self.lidar_to_camera_T.reshape(-1,1)) # (3, N)


        # Project to 2D and filter points within image boundaries
        # points2D = [ CAMERA_MODEL.project3dToPixel(point) for point in points3D[:, :3] ]
        # points2D = np.asarray(points2D)
        points2D = self.camera_matrix.dot(points3D_T).T
        points2D[:,0] /= points2D[:,2]
        points2D[:,1] /= points2D[:,2]
        points2D = points2D[:,:2]
        inrange = np.where((points2D[:, 0] >= 0) &
                        (points2D[:, 1] >= 0) &
                        (points2D[:, 0] < img.shape[1]) &
                        (points2D[:, 1] < img.shape[0]))
        points2D = points2D[inrange[0]].round().astype('int')

        # Draw the projected 2D points
        for i in range(len(points2D)):
            cv2.circle(img, tuple(points2D[i]), 2, tuple(colors[i]), -1)

        # Publish the projected points image
        try:
            image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e: 
            rospy.logerr(e)
        # rospy.loginfo("lidar image published.")
        # print("\rlidar image published.", end="")
        # sys.stdout.flush()

    '''
    Callback function to publish project image and run calibration

    Inputs:
        image - [sensor_msgs/Image] - ROS sensor image message
        camera_info - [sensor_msgs/CameraInfo] - ROS sensor camera info message
        velodyne - [sensor_msgs/PointCloud2] - ROS velodyne PCL2 message
        image_pub - [sensor_msgs/Image] - ROS image publisher

    Outputs: None
    '''
    def camera_lidar_callback(self, image_msg, pointcloud_msg, image_pub=None):
        global CAMERA_MODEL, FIRST_TIME, PAUSE, TF_BUFFER, TF_LISTENER, CAMERA_MODEL_INITED, PROJECT_MODE, WATCH_LIVESTREAM_OUTCOME, REMOVE_LAST_FRAME

        if FIRST_TIME:
            FIRST_TIME = False
            # TF listener
            # rospy.loginfo('Setting up static transform listener')
            # TF_BUFFER = tf2_ros.Buffer()
            # TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)
            if (IMAGE_COMPRESSED):
                img = CV_BRIDGE.compressed_imgmsg_to_cv2(image_msg, 'bgr8')
                print("[img.shape]:", img.shape)
                self.image_shape = (img.shape[0], img.shape[1]) # (height, width)
            else:
                self.image_shape = (image_msg.height, image_msg.width)
            if (not os.path.exists(os.path.join(PKG_PATH, CALIB_PATH, self.camera_name+"_pcd"))):
                os.mkdir(os.path.join(PKG_PATH, CALIB_PATH, self.camera_name+"_pcd"))

        elif REMOVE_LAST_FRAME:
            REMOVE_LAST_FRAME = False
            if (len(self.lidar_points)>0):
                self.lidar_points.pop()
                self.board_points.pop()
                self.pixel_points.pop()
                self.board_points_num = len(self.board_points)
                self.pixel_points_num = len(self.pixel_points)
                self.lidar_points_num = len(self.lidar_points)
                rospy.loginfo("Current number of frames: %d" % len(self.lidar_points))
            else:
                rospy.logwarn("No frame stored.")
            start_keyboard_handler()

        # Projection/display mode
        elif PROJECT_MODE or (CAMERA_MODEL_INITED and WATCH_LIVESTREAM_OUTCOME):
            # rospy.loginfo("Entering project_point_cloud")
            self.project_point_cloud(pointcloud_msg, image_msg, image_pub)
            if self.calibrate_mode:
                PROJECT_MODE = False
            if (WATCH_LIVESTREAM_OUTCOME and PAUSE):
                PAUSE = False
                start_keyboard_handler()

        # Calibration mode
        elif PAUSE:
            rospy.loginfo("Frame %d" % self.frame_count)

            self.board_points_num = len(self.board_points)
            self.pixel_points_num = len(self.pixel_points)
            self.lidar_points_num = len(self.lidar_points)

            # Create GUI processes
            now = rospy.get_rostime()
            # proc_pool = multiprocessing.Pool(processes=2)
            # proc_results = []
            self.proc_results = multiprocessing.Queue()
            # proc_results.append(proc_pool.apply_async(self.extract_points_2D, args=(image_msg, now, self.board_points, self.pixel_points,)))
            img_proc = multiprocessing.Process(target=self.extract_points_2D, args=[image_msg, now, self.proc_results, self])
            self.procs.append(img_proc)
            # proc_results.append(proc_pool.apply_async(self.extract_points_3D, args=(pointcloud_msg, now, self.lidar_points,)))
            pcl_proc = multiprocessing.Process(target=self.extract_points_3D, args=[pointcloud_msg, now, self.proc_results, self])
            self.procs.append(pcl_proc)
            # pool.close()  # 关闭进程池，不再接受新的进程
            # pool.join()  # 主进程阻塞等待子进程的退出
            rospy.loginfo("Starting sub threads.")
            img_proc.start(); pcl_proc.start()
            img_proc.join(); pcl_proc.join()
            rospy.loginfo("All sub threads ended.")
            results = []
            results_valid = True
            if (self.proc_results.empty()):
                results_valid = False
            else:
                try:
                    for proc in self.procs:
                        results.append(self.proc_results.get_nowait())
                except Exception:
                    rospy.logwarn("Get results error. Pass this frame.")
                    results_valid = False
            if (results_valid and len(results)>1):
                if (len(results[0])==1 and len(results[1])==2):
                    self.lidar_points.append(results[0][0])
                    self.board_points.append(results[1][0])
                    self.pixel_points.append(results[1][1])
                elif (len(results[1])==1 and len(results[0])==2):
                    self.lidar_points.append(results[1][0])
                    self.board_points.append(results[0][0])
                    self.pixel_points.append(results[0][1])
                self.frame_count += 1
                
            print("current number of middle varaibles: board_points %d, pixel_points %d, lidar_points %d" % (len(self.board_points), len(self.pixel_points), len(self.lidar_points)))
            del self.procs
            self.procs = []

            # Calibrate for existing corresponding points
            if (self.frame_count >= self.calibrate_min_frames and (self.board_points_num < len(self.board_points))):
                ret, self.camera_matrix, self.dist_coeffs, self.chessboard_to_camera_rvecs, self.chessboard_to_camera_tvecs = cv2.calibrateCamera(self.board_points, self.pixel_points, self.image_shape,None,None)
                # 重投影误差
                total_error = 0
                for i in range(len(self.board_points)):
                    imgpoints2, _ = cv2.projectPoints(self.board_points[i], self.chessboard_to_camera_rvecs[i], self.chessboard_to_camera_tvecs[i], self.camera_matrix, self.dist_coeffs)
                    error = cv2.norm(np.array(self.pixel_points[i]), np.squeeze(imgpoints2), cv2.NORM_L2)/len(imgpoints2)
                    total_error += error
                print("reprojection error for current camera calibration: ", total_error/len(self.board_points))
                # save current camera calibration to file
                self.R = np.eye(3, dtype=np.float64)
                self.P = np.zeros((3, 4), dtype=np.float64)
                ncm, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, self.image_shape, 0)
                for j in range(3):
                    for i in range(3):
                        self.P[j,i] = ncm[j, i]
                calibration_string = cameraCalibrationYamlBuf(self.dist_coeffs, self.camera_matrix, self.R, self.P, self.image_shape, name=self.camera_name)
                if (not os.path.exists(self.folder)):
                    os.mkdir(self.folder)
                with open(os.path.join(self.folder, "intrinsics_"+self.camera_name+".yaml"), 'w') as f:
                    f.write(calibration_string)

                CAMERA_MODEL_INITED = True
                CAMERA_MODEL.K = mkmat(3, 3, self.camera_matrix)
                if (self.dist_coeffs is not None):
                    CAMERA_MODEL.D = mkmat(len(self.dist_coeffs.squeeze()), 1, self.dist_coeffs)
                else:
                    CAMERA_MODEL.D = None
                CAMERA_MODEL.R = mkmat(3, 3, self.R)
                CAMERA_MODEL.P = mkmat(3, 4, self.P)
                CAMERA_MODEL.full_K = mkmat(3, 3, self.camera_matrix)
                CAMERA_MODEL.full_P = mkmat(3, 4, self.P)
                CAMERA_MODEL.width = self.image_shape[1]
                CAMERA_MODEL.height = self.image_shape[0]
                CAMERA_MODEL.resolution = (CAMERA_MODEL.width, CAMERA_MODEL.height)
                CAMERA_MODEL.tf_frame = self.camera_name

                self.chessboard_to_camera_R = [cv2.Rodrigues(x)[0] for x in self.chessboard_to_camera_rvecs]
                rospy.loginfo("Entering calibration")
                self.calibrate()
            if (self.success):
                self.calibration_finished = True
                PROJECT_MODE = True

            # Resume listener
            with KEY_LOCK: PAUSE = False
            start_keyboard_handler()
            

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibrate", action="store_true", help="enable calibration mode")
    parser.add_argument('--project', action="store_true", help="enable projection mode")
    parser.add_argument('--image_topic', type=str, default='/rgb_right/image_raw/compressed')
    parser.add_argument('--lidar_name', type=str, default='HDL32')
    parser.add_argument('--transform_pointcloud', action="store_true")
    parser.add_argument('--pointcloud_topic', type=str, default='/lidar_cloud_calibrated')
    parser.add_argument('--camera_lidar_topic', type=str, default='/camera_lidar_topic', help="topic name of image message on which lidar pointcloud is projected")
    parser.add_argument('--chessboard_size', type=str, default='7x9', help="(number of corners in a row)x(number of corners in a column)")
    parser.add_argument('--square_size', type=float, default='0.12', help="length (in meter) of a side of square")
    parser.add_argument('--lidar_points', type=int, default='500', help="(ADVANCED) number of points choosen on chessboard plane. reduce this number could help this program run faster")
    parser.add_argument('--calibrate_min_frames', type=int, default='10', help="(ADVANCED) least number of frames to perform calibration")
    parser.add_argument('--use_L1_error', action="store_true", help="(ADVANCED) use L1 error for lidar and camera projection error")
    parser.add_argument('--tracking_mode', action="store_true", help="(ADVANCED) unimplemented. track the chessboard in pointcloud")
    args = parser.parse_args()

    # Calibration mode
    if args.calibrate:
        image_topic = args.image_topic
        pointcloud_topic = args.pointcloud_topic
        camera_lidar_topic = args.camera_lidar_topic
        PROJECT_MODE = False
    # Projection mode
    else:
        image_topic = args.image_topic
        pointcloud_topic = args.pointcloud_topic
        camera_lidar_topic = args.camera_lidar_topic
        PROJECT_MODE = args.project

    # Start CameraLiDARCalibrator
    if ("compressed" in image_topic):
        IMAGE_COMPRESSED = True
    calibrator = CameraLiDARCalibrator(args, image_topic, pointcloud_topic, camera_lidar_topic)
