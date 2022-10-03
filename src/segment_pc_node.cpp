#include "segpc_yolov7/segment_pc_node.h"

PcSegmenter::PcSegmenter(ros::NodeHandle& nh)
{
    PcSegmenter::nh = nh;
    ros::param::get("/mask_topic", PcSegmenter::roiTopic);
    ros::param::get("/image_topic", PcSegmenter::imageTopic );
    ros::param::get("/camera_info_topic", PcSegmenter::camInfoTopic);
    ros::param::get("/publish_topic", PcSegmenter::pcTopic);
    PcSegmenter::roiAcquired = false;   // flag to prevent publishing before roi is acquired
    PcSegmenter::imageAcquired = false; // flag to prevent publishing before depth image is acquired
    PcSegmenter::cameraInfoSub = PcSegmenter::nh.subscribe(PcSegmenter::camInfoTopic,1, &PcSegmenter::cbCameraInfo, this);
    // PcSegmenter:pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(PcSegmenter::pcTopic, 1);
    PcSegmenter:pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(PcSegmenter::pcTopic, 5);
    this->colorAcquired = false;
    
    PcSegmenter::classList = {"person", "chair", "tvmonitor", "bottle", "cell phone"};
    PcSegmenter::colors = {{255, 255, 255}, {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 0, 255}};
    PcSegmenter::colorDict = { {classList[0], colors[0]},
                                {classList[1], colors[1]},
                                {classList[2], colors[2]},
                                {classList[3], colors[3]},
                                {classList[4], colors[4]}
    };
}       

/*
callback to update stored region of interest
nonblocking lock is used to prevent cbRoi() from overwriting while publishPc() is reading, 
but roiSub keeps running
params: yolov7_ros/DetectionDataArray
*/
void PcSegmenter::cbRoi(const yolov7_ros::DetectionDataArrayConstPtr& msg) 
{
    if(PcSegmenter::lock.try_lock())
    { 
        PcSegmenter::objDataList = msg->objects;
        PcSegmenter::roiAcquired = true;
        PcSegmenter::lock.unlock(); // release the lock 
    }
}

/*
callback to update stored depth image
nonblocking lock is used to prevent cbDepthImage() from overwriting while publishPc() is reading, 
but imageSub keeps running
params: sensor_msgs/Image
*/
void PcSegmenter::cbDepthImage(const sensor_msgs::ImageConstPtr& msg) 
{
    if(PcSegmenter::lock.try_lock())
    { 
        //convert ROS sensor image msg to cv::Mat 
        PcSegmenter::depthImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image; // depth in mm as uint16
        PcSegmenter::imageAcquired = true;
        PcSegmenter::lock.unlock(); // release the lock 
    }
}

void PcSegmenter::cbColorImage(const sensor_msgs::ImageConstPtr& msg) 
{
    if(this->lock.try_lock())
    { 
        //convert ROS sensor image msg to cv::Mat 
        this->colorImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3)->image; // depth in mm as uint16
        this->colorAcquired = true;
        this->lock.unlock(); // release the lock 
    }
}

/*
callback to get camera intrinsics and form inverse of camera matrix
params: sensor_msgs/camera_info
*/
void PcSegmenter::cbCameraInfo(const sensor_msgs::CameraInfo& msg) 
{
    // extract camera instrinsics parameters
    PcSegmenter::camFrameID = msg.header.frame_id;
    float fx = (float)msg.K[0];
    float fy = (float)msg.K[4];
    float ppx = (float)msg.K[2];
    float ppy = (float)msg.K[5];

    // calculate camera matrix inverse, store parameters as float64
    PcSegmenter::camMatInv = (cv::Mat_<float>(2,3) << 1/fx, 0, -ppx/fx, 0, 1/fy, -ppy/fy);    // assume skew is zero

    // unregister camera info subscriber
    PcSegmenter::cameraInfoSub.shutdown();

    // start bounding box and depth image subscriber threads
    PcSegmenter::roiSub = PcSegmenter::nh.subscribe(PcSegmenter::roiTopic, 1, &PcSegmenter::cbRoi, this); 
    PcSegmenter::imageSub = PcSegmenter::nh.subscribe(PcSegmenter::imageTopic, 1, &PcSegmenter::cbDepthImage, this);
    this->colorSub = PcSegmenter::nh.subscribe("/camera/color/image_raw", 1, &PcSegmenter::cbColorImage, this);

}

/*
callback to publish pointcloud using roi, depthImage, and cameraInfo
acquires lock to read roi, depthImage, and cameraInfo
call to acquire lock is blocking
*/
void PcSegmenter::publishPc()
{
    // check if roi and image have been acquired first
    if (PcSegmenter::roiAcquired == true && PcSegmenter::imageAcquired == true && this->colorAcquired == true) 
    {
        
        // read newest data
        PcSegmenter::lock.lock();
        cv::Mat inputImage = PcSegmenter::depthImage.clone();
        std::vector<yolov7_ros::ObjectData> objDataList = PcSegmenter::objDataList;
        cv::Mat inputColor = this->colorImage.clone();
        PcSegmenter::lock.unlock();

        // iterate through all detected objects
        cv::Mat compositeMask = cv_bridge::toCvCopy(objDataList[0].mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;
        for (unsigned int i = 1; i < objDataList.size(); i++) 
        {
            //bitwise_or to combine masks
            cv::Mat tempCompositeMask = compositeMask.clone();
            cv::Mat currMask = cv_bridge::toCvCopy(objDataList[i].mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;
            cv::bitwise_or(tempCompositeMask, currMask, compositeMask);
        }

        // deproject depth image
        // pcl::PointCloud<pcl::PointXYZ>cloud;
        pcl::PointCloud<pcl::PointXYZRGB>cloud;
        for (unsigned int u = 0; u < compositeMask.rows; u++)
        {
            for (unsigned int v = 0; v < compositeMask.cols; v++)
            {
                if (compositeMask.at<unsigned char>(u,v) > 0 && inputImage.at<unsigned int>(u,v) > 30 && inputImage.at<unsigned int>(u,v) < 10000)  // ignore min max depth values and pixels outside of mask
                {
                    // pcl::PointXYZ point;
                    pcl::PointXYZRGB point;
                    float z = (float)inputImage.at<unsigned int>(u,v)/1000;   // convert depth from mm to m
                    float x = (PcSegmenter::camMatInv.at<float>(0,0)*((float)v) + PcSegmenter::camMatInv.at<float>(0,2))*z; 
                    float y = (PcSegmenter::camMatInv.at<float>(1,1)*((float)u) + PcSegmenter::camMatInv.at<float>(1,2))*z;
                    // xyz is rotated to transform from camera frame coords to TF of camera
                    point.x = z;
                    point.y = -x;
                    point.z = -y;
                    std::uint32_t rgb = (static_cast<std::uint32_t>(inputColor.at<cv::Vec3b>(u,v)[0]) << 16 | 
                                         static_cast<std::uint32_t>(inputColor.at<cv::Vec3b>(u,v)[1]) << 8 | static_cast<std::uint32_t>(inputColor.at<cv::Vec3b>(u,v)[2]));
                    point.rgb = *reinterpret_cast<float*>(&rgb);
                    cloud.points.push_back(point); 
                }
            }
        }
        cloud.header.frame_id = PcSegmenter::camFrameID;
        pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
        PcSegmenter::pub.publish(cloud);
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segment_pc_node");
    ros::NodeHandle nh;
    PcSegmenter segmentPcNode(nh);
    float pubRate;
    ros::param::get("/publish_rate", pubRate);
    ros::Rate loopRate(pubRate);
    while (ros::ok())
    {
        segmentPcNode.publishPc();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
