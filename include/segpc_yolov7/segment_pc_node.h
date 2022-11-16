#ifndef SEGMENT_PC_NODE_H
#define SEGMENT_PC_NODE_H

// ROS
#include "ros/ros.h"
// yolov7_ros
#include "yolov7_ros/DetectionDataArray.h"
#include "yolov7_ros/ObjectData.h"
// images and opencv
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h" 
#include "sensor_msgs/CameraInfo.h"
// pointcloud and pcl
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// thread locking
#include <mutex>   
// primitives and storage
#include <iostream>
#include <map>
#include <vector>

class PcSegmenter
{
    private:
        ros::NodeHandle nh;
        std::mutex lock;

        std::string roiTopic;
        ros::Subscriber roiSub;
        std::vector<yolov7_ros::ObjectData> objDataList;
        bool roiAcquired;

        std::string depthTopic;
        ros::Subscriber depthSub;
        cv::Mat depthImage;
        bool depthAcquired;

        std::string colorTopic;
        ros::Subscriber colorSub;
        cv::Mat colorImage;
        bool colorAcquired;

        std::string camInfoTopic;
        ros::Subscriber cameraInfoSub;
        cv::Mat camMatInv;
        std::string camFrameID;
        
        ros::Publisher pub;
        std::string pcTopic;
        
        ushort minDepth;
        ushort maxDepth;
        std::vector<std::string> classList;
        std::vector<std::vector<unsigned char>> colors;
        std::map<std::string, std::vector<unsigned char>> colorDict;
    public:
        PcSegmenter(ros::NodeHandle& nh);
        void cbRoi(const yolov7_ros::DetectionDataArrayConstPtr& detectionData);
        void cbDepthImage(const sensor_msgs::ImageConstPtr& msg);
        void cbColorImage(const sensor_msgs::ImageConstPtr& msg);
        void cbCameraInfo(const sensor_msgs::CameraInfo& msg);
        cv::Mat combineMask(std::vector<yolov7_ros::ObjectData>& inputObjData);
        std::vector<cv::Point2f> getShiTomasi(cv::Mat& inputColor, cv::Mat& mask, int maxCorners);
        void publishPc();
    
};

#endif