bit-ivrc多传感器标定方法
========================

Edit by sundong

1、激光雷达和惯导标定
------------------
先采集惯导和激光雷达的bag包。  
将`iv_calibraton.launch`中的`pub_imu_lidar`改为`1`。
```
roslaunch iv_calibraton.launch
rosbag record /Imu_vel /lidar_cloud_origin
```
将`iv_calibraton.launch`中的`bag_file`改为保存的bag位置，
将`iv_calibraton.launch`中的`pub_imu_lidar`改为`1`，`pub_imu_lidar`改为`0`。
```
roslaunch iv_calibraton.launch
```
可以在`calibrationlidar/src/lidar_align/results`看到标定结果。

2、多激光雷达标定
--------------
首先，确定xml文件中的`calibrationmode`的值为`1`。  
将`iv_calibraton.launch`中的`save_pcd`改为`1`。
```
catkin build
source devel/setup.zsh
roslaunch iv_calibraton.launch
````
驾驶车辆围绕特征点较多的地点一圈，绕圈结束后关闭程序，  
可以看到data/calibration文件夹下存储了对应时间点的文件，    
这个文件中存储了生成的各个雷达的pcd文件，文件夹“0”代表第一雷达，以此类推。

一般来说，将0雷达作为基础雷达（base_lidar,建议为车前方的那个），将“0”文件夹内的所有“.pcd”文件复制到`/src/lidar_automatic_calibration/data/Base_LiDAR_Frames`文件夹下面，将需要标定的雷达（如“1”或“2”）文件夹下的“.pcd”文件复制到`/src/lidar_automatic_calibration/data/Target-LiDAR-Frames`文件夹下面。  
将`/src/lidar_automatic_calibration/data`中的`base_lidar.lua`中的数填入，这个数据是前面激光雷达和惯导标定的数据。  
用尺子量出需要标定的雷达和基础雷达的位置，和大概的角度，填入`target2base_lidar.lua`中，这个数只是用来计算相对位置时迭代的初值，不需要很准。
下面进行
```
roslaunch iv_calibraton.launch
```
运行时间较久，耐心等待。

3、单激光雷达
-----------------
将`iv_calibraton.launch`中的`cali_single_lidar`改为`1`。
```
roslaunch iv_calibraton.launch
```
暂没有进行独立尝试，细节以后更新。

4、激光雷达和相机标定
-----------------
将`iv_calibraton.launch`中的`cali_lidar_camera`改为`1`。
```
roslaunch iv_calibraton.launch
```
暂没有进行独立尝试，细节以后更新。

