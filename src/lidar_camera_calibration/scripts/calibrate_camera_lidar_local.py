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
1. To perform calibration using the GUI to pick correspondences:

    Prepare the corresponding images and pointclouds in a folder, like the following:
    - PKG_PATH/calibration_data/lidar_camera_calibration/[camera_name]_pcd/[0...n].jpg
    - PKG_PATH/calibration_data/lidar_camera_calibration/[camera_name]_pcd/[0...n].pcd
    
    $ rosrun lidar_camera_calibration calibrate_camera_lidar_local.py --calibrate \
        --path XXX --image_name XXX --pcd_name XXX [--chessboard_size 7x9 \
        --square_size 0.12 --lidar_points 500 --calibrate_min_frames 6]

    The calibrate extrinsic are saved as following:
    - PKG_PATH/calibration_data/lidar_camera_calibration/extrinsics.npz
    --> 'euler' : euler angles (3, )
    --> 'R'     : rotation matrix (3, 3)
    --> 'T'     : translation offsets (3, )

2. To display the LiDAR points projected on to the camera plane:

    $ rosrun lidar_camera_calibration calibrate_camera_lidar_local.py --project \
        --path XXX --image_name XXX --pcd_name XXX

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
import matplotlib.pyplot as plt
import yaml
import argparse
import glob
from scipy import optimize
# ROS modules
PKG = 'lidar_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import rosbag
import rospy
# import tf2_ros
import ros_numpy
import image_geometry
import message_filters
from cv_bridge import CvBridge, CvBridgeError
# from tf.transformations import euler_from_matrix
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
# Global paths
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CALIB_PATH = 'calibration_data/lidar_camera_calibration'


'''
Keyboard handler thread
Inputs: None
Outputs: None
'''
def handle_keyboard():
    global KEY_LOCK, PAUSE
    key = six.moves.input('Press [ENTER] to pause and pick points\n')
    if (key == 'q'):
        print("Exit.")
        sys.exit()
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


class CameraLiDARLocalCalibrator:
    '''
    The main ROS node which handles the topics

    Inputs:
        path - [str] image and pcd file folder
        image_filename - [str] - image file name
        pcd_filename - [str] - pcd file name

    Outputs: None
    '''
    def __init__(self, args, path, image_filename, pcd_filename):
        self.args = args
        # Start node
        rospy.init_node('calibrate_camera_lidar', anonymous=True)
        rospy.loginfo('Current PID: [%d]' % os.getpid())
        rospy.loginfo('Projection mode: %s' % PROJECT_MODE)
        rospy.loginfo('path: %s' % path)
        rospy.loginfo('image_filename: %s' % image_filename)
        rospy.loginfo('pcd_filename: %s' % pcd_filename)
        assert ('x' in args.chessboard_size)
        self.chessboard_size = tuple([int(x) for x in args.chessboard_size.split('x')])
        assert (len(self.chessboard_size) == 2)
        self.chessboard_diagonal = np.sqrt(self.chessboard_size[0]**2+self.chessboard_size[1]**2) * args.square_size
        # chessboard refine termination criteria
        self.chessboard_corner_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.chessboard_corner_coords = np.zeros((self.chessboard_size[0]*self.chessboard_size[1],3), np.float32)
        self.chessboard_corner_coords[:,:2] = np.mgrid[0:self.chessboard_size[0],0:self.chessboard_size[1]].T.reshape(-1,2) * args.square_size
        # print("self.chessboard_corner_coords:", self.chessboard_corner_coords.shape)
        # print(self.chessboard_corner_coords)
        self.image_shape = None
        self.board_points = []
        self.pixel_points = []
        self.lidar_points = []
        self.chessboard_to_camera_rvecs = []
        self.chessboard_to_camera_R = []
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
        self.path = path
        self.image_filename = image_filename
        self.pcd_filename = pcd_filename
        self.image_files = glob.glob(os.path.join(path, "*.jpg"))
        self.pcd_files = glob.glob(os.path.join(path, "*.pcd"))
        self.image_shape = cv2.imread(self.image_files[0]).shape[:2]
        self.camera_name = path.strip('/').split('/')[-1]
        self.camera_matrix
        # assert len(self.image_files) == len(self.pcd_files)
        # print(self.image_files)
        # print(self.pcd_files)
        rospy.loginfo("chessboard_size: %s" % str(self.chessboard_size))
        rospy.loginfo("chessboard_diagonal: %.2f" % (self.chessboard_diagonal))
        rospy.loginfo("use_L1_error: %s" % str(self.use_L1_error))

        if PROJECT_MODE:
            self.camera_lidar_fuse()
        else:
            self.camera_lidar_read()

    '''
    Runs the image point selection GUI process

    Inputs:
        img_file - [string] - image file name
        rectify - [bool] - to specify whether to rectify image or not

    Outputs:
        Picked points saved in PKG_PATH/CALIB_PATH/img_corners.npy
    '''
    def extract_points_2D(self, img_file, proc_results, rectify=False):
        # print("board_points:", len(board_points), ":", [x.shape for x in board_points])
        # Log PID
        rospy.loginfo('2D Picker PID: [%d] file name: %s' % (os.getpid(), os.path.basename(img_file)))

        # Read image
        img = cv2.imread(img_file)

        # Rectify image
        if rectify and CAMERA_MODEL_INITED: CAMERA_MODEL.rectifyImage(img, img)

        # detect corners
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        # corners2 = None
        if ret == True:
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.chessboard_corner_criteria)
            # Draw and display the corners
            cv2.drawChessboardCorners(img, self.chessboard_size, corners, ret)
            cv2.namedWindow('chessboard', 0)
            cv2.imshow('chessboard', img)
            cv2.resizeWindow('chessboard', 640,480)
            cv2.moveWindow('chessboard', 500,0)
            key = cv2.waitKey(0)
            if (key == ord('q')):
                # cv2.destroyAllWindows()
                pass
            cv2.destroyWindow('chessboard')
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
        pcd_file - [string] - pcd file name

    Outputs:
        Picked points saved in PKG_PATH/CALIB_PATH/pcl_corners.npy
    '''
    def extract_points_3D(self, pcd_file, proc_results):
        # print("lidar_points:", len(lidar_points), ":", [x.shape for x in lidar_points])
        # Log PID
        rospy.loginfo('3D Picker PID: [%d] file name: %s' % (os.getpid(), os.path.basename(pcd_file)))

        # Extract points data
        points = pcl.load(pcd_file).to_array()
        # Select points within chessboard range
        inrange = np.where((np.abs(points[:, 0]) < 8) &
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
            rospy.logwarn("No point selected! Skip this frame.")
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
        # 1. linear calibration
        if (self.lidar_to_camera_R is None or self.lidar_to_camera_T is None):
            rospy.loginfo("Step 1: Linear calibration.")
            total_lidar_points = np.row_stack(self.lidar_points)
            mean_lidar_points = total_lidar_points.mean(axis=0)
            diff_lidar_points = total_lidar_points - mean_lidar_points
            scale_lidar_points = np.abs(diff_lidar_points).max(axis=0).mean()
            A = None
            b = None
            for i in range(self.frame_count):
                lidar_points = (self.lidar_points[i] - mean_lidar_points) / scale_lidar_points
                r3 = self.chessboard_to_camera_R[i][:,2].reshape(-1,1) # last column of R (normal vector of chessboard plane in camera coordinate)
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
        print("optimized value [rvec_and_tvec]:", rvec_and_tvec)
        print("optimization success status:", self.success)
        self.lidar_to_camera_R = cv2.Rodrigues(rvec_and_tvec[0:3])[0]
        self.lidar_to_camera_T = rvec_and_tvec[3:6].reshape(-1,1)
        print("R:", self.lidar_to_camera_R)
        print("T:", self.lidar_to_camera_T)

        euler = euler_from_matrix(self.lidar_to_camera_R)
        
        # Save extrinsics
        np.savez(os.path.join(self.folder, 'extrinsics.npz'),
            euler=euler, R=self.lidar_to_camera_R, T=self.lidar_to_camera_T.T)
        writeYAMLFile(os.path.join(self.folder, 'extrinsics.yaml'), {'euler':list(euler), 'R':self.lidar_to_camera_R, 'T':self.lidar_to_camera_T.T})


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
    def project_point_cloud(self, pcd_filename, img_filename, image_pub):
        # Read image
        print(os.path.basename(pcd_filename), os.path.basename(img_filename))
        img = cv2.imread(img_filename)

        # Transform the point cloud
        # try:
        #     transform = TF_BUFFER.lookup_transform('world', 'vehicle_frame', rospy.Time())
        #     pcd_filename = do_transform_cloud(pcd_filename, transform)
        # except tf2_ros.LookupException:
        #     pass

        # Extract points from message
        if (self.lidar_to_camera_R is None  or self.lidar_to_camera_T is None):
            rospy.loginfo("Reading extrinsics from %s" % os.path.join(self.folder, 'extrinsics.yaml'))
            extrinsics = readYAMLFile(os.path.join(self.folder, 'extrinsics.yaml'))
            self.lidar_to_camera_R = np.array(extrinsics['R'])
            print("lidar_to_camera_R:", self.lidar_to_camera_R)
            self.lidar_to_camera_T = np.array(extrinsics['T']) #/100.
            print("lidar_to_camera_T:", self.lidar_to_camera_T)
        if (self.camera_matrix is None):
            intrinsics = readYAMLFile(os.path.join(self.folder, "intrinsics_"+self.camera_name+".yaml"))
            self.camera_matrix = np.array(intrinsics['camera_matrix']['data']).reshape(3,3)

        points3D = pcl.load(pcd_filename).to_array()
        
        # Filter points in front of camera
        inrange = np.where((np.abs(points3D[:, 0]) < 30) &
                           (points3D[:, 1] > 0.2) & (points3D[:,1] < 50) & (points3D[:,2] < 6) & (points3D[:,2] > 0))
        points3D = points3D[inrange[0]]
        max_intensity = np.max(points3D[:, -1])
        points3D_T = (self.lidar_to_camera_R.dot(points3D.T) + self.lidar_to_camera_T.reshape(-1,1)) # (3, N)

        # Color map for the points
        cmap = matplotlib.cm.get_cmap('jet')
        colors = cmap(points3D[:, -1] / max_intensity) * 255

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
        # print("points loaded:", points3D.shape, ". points fall on image:", points2D.shape)

        # Draw the projected 2D points
        for i in range(len(points2D)):
            cv2.circle(img, tuple(points2D[i]), 2, tuple(colors[i]), -1)

        plt.imshow(img[:,:,::-1])
        plt.title('fusion')
        plt.xticks([]), plt.yticks([])
        plt.show()

    '''
    Callback function to publish project image and run calibration

    Inputs:
        image - [sensor_msgs/Image] - ROS sensor image message
        camera_info - [sensor_msgs/CameraInfo] - ROS sensor camera info message
        velodyne - [sensor_msgs/PointCloud2] - ROS velodyne PCL2 message
        image_pub - [sensor_msgs/Image] - ROS image publisher

    Outputs: None
    '''
    def camera_lidar_read(self, image_pub=None):
        global CAMERA_MODEL, FIRST_TIME, PAUSE, TF_BUFFER, TF_LISTENER, CAMERA_MODEL_INITED, PROJECT_MODE

        # # Projection/display mode
        # if CAMERA_MODEL_INITED and PROJECT_MODE:
        #     rospy.loginfo("Entering project_point_cloud")
        #     self.project_point_cloud(pointcloud_msg, image_msg, image_pub)
        #     PROJECT_MODE = False

        # Calibration mode
        # elif PAUSE:
        i = 1
        while (i < len(self.pcd_files) + 1):

            # image_filename = self.image_files[i]
            image_filename = os.path.join(self.path, self.image_filename % i)
            # pcd_filename = self.pcd_files[i]
            pcd_filename = os.path.join(self.path, self.pcd_filename % i)
            rospy.loginfo("Frame %d. image file: %s. pcd file: %s" % (i, os.path.basename(image_filename), os.path.basename(pcd_filename)))

            if CAMERA_MODEL_INITED and PROJECT_MODE:
                rospy.loginfo("Entering project_point_cloud")
                self.project_point_cloud(pcd_filename, image_filename, image_pub)
                PROJECT_MODE = False

            self.board_points_num = len(self.board_points)
            self.pixel_points_num = len(self.pixel_points)
            self.lidar_points_num = len(self.lidar_points)

            # Create GUI processes
            now = rospy.get_rostime()
            # proc_pool = multiprocessing.Pool(processes=2)
            # proc_results = []
            self.proc_results = multiprocessing.Queue()
            # proc_results.append(proc_pool.apply_async(self.extract_points_3D, args=(pointcloud_msg, self.lidar_points,)))
            pcl_proc = multiprocessing.Process(target=self.extract_points_3D, args=[pcd_filename, self.proc_results])
            self.procs.append(pcl_proc)
            # proc_results.append(proc_pool.apply_async(self.extract_points_2D, args=(image_msg, self.board_points, self.pixel_points,)))
            img_proc = multiprocessing.Process(target=self.extract_points_2D, args=[image_filename, self.proc_results])
            self.procs.append(img_proc)
            # pool.close()  # 关闭进程池，不再接受新的进程
            # pool.join()  # 主进程阻塞等待子进程的退出
            # rospy.loginfo("Starting sub threads.")
            img_proc.start(); pcl_proc.start()
            img_proc.join(); pcl_proc.join()
            # rospy.loginfo("All sub threads ended.")
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
            if (self.frame_count >= self.calibrate_min_frames and CAMERA_MODEL_INITED and (self.board_points_num < len(self.board_points))):
                ret, self.camera_matrix, self.dist_coeffs, self.chessboard_to_camera_rvecs, self.chessboard_to_camera_tvecs = cv2.calibrateCamera(self.board_points, self.pixel_points, self.image_shape,None,None)
                # 重投影误差
                total_error = 0
                for i in range(len(self.board_points)):
                    imgpoints2, _ = cv2.projectPoints(self.board_points[i], self.chessboard_to_camera_rvecs[i], self.chessboard_to_camera_tvecs[i], self.camera_matrix, self.dist_coeffs)
                    error = cv2.norm(np.array(self.pixel_points[i]), np.squeeze(imgpoints2), cv2.NORM_L2)/len(imgpoints2)
                    total_error += error
                print("reprojection error for current camera calibration: ", total_error/len(self.board_points))
                print("self.dist_coeffs:", self.dist_coeffs)
                # save current camera calibration to file
                self.R = np.eye(3, dtype=np.float64)
                self.P = np.zeros((3, 4), dtype=np.float64)
                ncm, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, self.image_shape, 0)
                for j in range(3):
                    for i in range(3):
                        self.P[j,i] = ncm[j, i]
                calibration_string = cameraCalibrationYamlBuf(self.dist_coeffs, self.camera_matrix, self.R, self.P, self.image_shape, name=self.camera_name)
                if (not os.path.exists(os.path.join(PKG_PATH, CALIB_PATH))):
                    os.mkdir(os.path.join(PKG_PATH, CALIB_PATH))
                with open(os.path.join(PKG_PATH, CALIB_PATH, "intrinsics_"+self.camera_name+".yaml"), 'a') as f:
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

            key = six.moves.input('Press [ENTER] to continue or [q] to quit or [r] to process current frame again: ')
            if (key == 'q'):
                if (key == 'q'):
                    print("Exit.")
                    sys.exit()
            elif (key != 'r'):
                i += 1
                print("[ENTER] pressed. Process next frame.")
            else:
                print("Process frame %d again" % i)
            # Resume listener
            # with KEY_LOCK: PAUSE = False
            # start_keyboard_handler()


    def camera_lidar_fuse(self, image_pub=None):
        while (not CAMERA_MODEL_INITED):
            time.sleep(1)
        for i in range(1, len(self.pcd_files) + 1):

            rospy.loginfo("Frame %d" % i)
            # image_filename = self.image_files[i]
            image_filename = os.path.join(self.path, self.image_filename % i)
            # pcd_filename = self.pcd_files[i]
            pcd_filename = os.path.join(self.path, self.pcd_filename % i)

            self.project_point_cloud(pcd_filename, image_filename, image_pub)
            # PROJECT_MODE = False


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibrate", action="store_true", help="enable calibration mode")
    parser.add_argument('--project', action="store_true", help="enable projection mode")
    parser.add_argument('--path', type=str, default='./')
    parser.add_argument('--image_name', type=str, default='*.png', help="image file name format")
    parser.add_argument('--pcd_name', type=str, default='*.pcd', help="pcd file name format")
    parser.add_argument('--chessboard_size', type=str, default='7x9', help="(number of corners in a row)x(number of corners in a column)")
    parser.add_argument('--square_size', type=float, default='0.12', help="length (in meter) of a side of square")
    parser.add_argument('--lidar_points', type=int, default='500', help="(ADVANCED) number of points choosen on chessboard plane. reduce this number could help this program run faster")
    parser.add_argument('--calibrate_min_frames', type=int, default='6', help="(ADVANCED) least number of frames to perform calibration")
    parser.add_argument('--use_L1_error', action="store_true", help="(ADVANCED) use L1 error for lidar and camera projection error")
    parser.add_argument('--tracking_mode', action="store_true", help="(ADVANCED) unimplemented. track the chessboard in pointcloud")
    args = parser.parse_args()

    path = args.path
    image_filename = args.image_name
    pcd_filename = args.pcd_name
    # Calibration mode
    if args.calibrate:
        PROJECT_MODE = False
    # Projection mode
    else:
        PROJECT_MODE = args.project

    # Start subscriber
    calibrator = CameraLiDARLocalCalibrator(args, path, image_filename, pcd_filename)
