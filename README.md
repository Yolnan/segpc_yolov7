# README #
Version 1.0
### Summary ###
This ROS node publishes a segmented pointcloud received from a realsense RGB-D camera and detection info from [yolov7_ros](https://cerlab-ugv@dev.azure.com/cerlab-ugv/Theia/_git/yolov7_ros).

### Set up ###
Tested on Ubuntu 20.04 running ROS Noetic

* Required ROS Packages: 
    * roscpp
    * sensor_msgs
    * cv_bridge
    * [yolov7_ros](https://cerlab-ugv@dev.azure.com/cerlab-ugv/Theia/_git/yolov7_ros)
    * [realsense-ros](https://github.com/IntelRealSense/realsense-ros.git)

* Dependencies:
    * OpenCV
    * PCL

### Running System ###
```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true color_width:=640 color_height:=480 color_fps:=30 depth_width:=640 depth_height:=480 depth_fps:=30
roslaunch yolov7_ros yolo-v7.launch
roslaunch segpc_yolov7 segpc_yolov7.launch
```
### Contact ###
* Yolnan Chen (MSME-R 2023): yolnanc@andrew.cmu.edu 
* Grant Metts (UGV PI/PhD): gmetts@andrew.cmu.edu