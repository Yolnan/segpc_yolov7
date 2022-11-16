#include "segpc_yolov7/segment_pc_node.h"

PcSegmenter::PcSegmenter(ros::NodeHandle& _nh): nh{_nh}
{
    ros::param::get("/mask_topic", roiTopic);
    ros::param::get("/depth_topic", depthTopic);
    ros::param::get("/color_topic", colorTopic );
    ros::param::get("/camera_info_topic", camInfoTopic);
    ros::param::get("/publish_topic", pcTopic);
    roiAcquired = false;   // flag to prevent publishing before roi is acquired
    depthAcquired = false; // flag to prevent publishing before depth image is acquired
    colorAcquired = false; // flag to prevent publishing before color image is acquired
    cameraInfoSub = nh.subscribe(camInfoTopic,1, &PcSegmenter::cbCameraInfo, this);
    // PcSegmenter:pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(pcTopic, 1);
    PcSegmenter:pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(pcTopic, 5);

    int minDepthInt, maxDepthInt;
    ros::param::get("/min_depth", minDepthInt);
    ros::param::get("/max_depth", maxDepthInt);
    minDepth = minDepthInt;
    maxDepth = maxDepthInt;
    
    classList = {"person", "chair", "tvmonitor", "bottle", "cell phone"};
    colors = {{255, 255, 255}, {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 0, 255}};
    colorDict = { {classList[0], colors[0]},
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
    if(lock.try_lock())
    { 
        objDataList = msg->objects;
        if ( objDataList.size() == 0 )
        {
            roiAcquired = false;
            std::cout << "empty object list acquired\n";
        }
        else{
            roiAcquired = true;
        }

        lock.unlock(); // release the lock 
    }
}

/*
callback to update stored depth image
nonblocking lock is used to prevent cbDepthImage() from overwriting while publishPc() is reading, 
but imageSub keeps running
params: sensor_msgs/Image
*/
void PcSegmenter::cbDepthImage(const sensor_msgs::ImageConstPtr &msg) 
{
    if(lock.try_lock())
    { 
        //convert ROS sensor image msg to cv::Mat 
        depthImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image; // depth in mm as uint16
        depthAcquired = true;
        lock.unlock(); // release the lock 
    }
}

void PcSegmenter::cbColorImage(const sensor_msgs::ImageConstPtr& msg) 
{
    if(lock.try_lock())
    { 
        //convert ROS sensor image msg to cv::Mat 
        colorImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3)->image; // depth in mm as uint16
        colorAcquired = true;
        lock.unlock(); // release the lock 
    }
}

/*
callback to get camera intrinsics and form inverse of camera matrix
params: sensor_msgs/camera_info
*/
void PcSegmenter::cbCameraInfo(const sensor_msgs::CameraInfo& msg) 
{
    // extract camera instrinsics parameters
    camFrameID = msg.header.frame_id;
    float fx = msg.K[0];
    float fy = msg.K[4];
    float ppx = msg.K[2];
    float ppy = msg.K[5];

    float a00 = 1/fx;
    float a01 = 0;
    float a03 = -ppx/fx;
    float a10 = 0;
    float a11 = 1/fy;
    float a12 = -ppy/fy;

    // calculate camera matrix inverse, store parameters as float64
    
    camMatInv = (cv::Mat_<float>(2,3) << a00, a01, a03, a10, a11, a12);    // assume skew is zero

    // unregister camera info subscriber
    cameraInfoSub.shutdown();

    // start bounding box and depth image subscriber threads
    roiSub = nh.subscribe(roiTopic, 1, &PcSegmenter::cbRoi, this); 
    depthSub = nh.subscribe(depthTopic, 1, &PcSegmenter::cbDepthImage, this);
    colorSub = nh.subscribe(colorTopic, 1, &PcSegmenter::cbColorImage, this);

}

/*
combine detection mask using bitwise_or
*/
cv::Mat PcSegmenter::combineMask(std::vector<yolov7_ros::ObjectData>& inputObjData)
{
    // iterate through all detected objects
    
    cv::Mat compositeMask = cv_bridge::toCvCopy(inputObjData[0].mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;

    for (unsigned int i = 1; i < inputObjData.size(); i++) 
    {
        //bitwise_or to combine masks
        cv::Mat tempCompositeMask = compositeMask.clone();
        cv::Mat currMask = cv_bridge::toCvCopy(inputObjData[i].mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;
        cv::bitwise_or(tempCompositeMask, currMask, compositeMask);
    }
    return compositeMask;
}

/*
extract shi-tomasi features from color image
*/
std::vector<cv::Point2f> PcSegmenter::getShiTomasi(cv::Mat& inputColor, cv::Mat& mask, int maxCorners) 
{
    //extract Shi-Tomasi features
    cv::Mat inputGray;
    cv::cvtColor(inputColor, inputGray, cv::COLOR_BGR2GRAY);
    maxCorners = MAX(maxCorners, 1);
    std::vector<cv::Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3, gradientSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    cv::goodFeaturesToTrack(inputGray,
                            corners,
                            maxCorners,
                            qualityLevel,
                            minDistance,
                            mask,
                            blockSize,
                            gradientSize,
                            useHarrisDetector,
                            k);
    return corners;

}
 
/*
callback to publish pointcloud using roi, depthImage, and cameraInfo
acquires lock to read roi, depthImage, and cameraInfo
call to acquire lock is blocking
*/
void PcSegmenter::publishPc()
{
    // check if roi and image have been acquired first
    if (roiAcquired == true && depthAcquired == true && colorAcquired == true) 
    {
        // read newest data
        lock.lock();
        
        cv::Mat inputDepth = depthImage.clone();
        std::vector<yolov7_ros::ObjectData> inputObjData = objDataList;
        cv::Mat inputColor = colorImage.clone();
        lock.unlock();
        roiAcquired = false;
        depthAcquired = false;
        colorAcquired = false;
        // combine masks
        cv::Mat compositeMask = combineMask(inputObjData);

        // extract Shi-Tomasi features
        std::vector<cv::Point2f> corners = getShiTomasi(inputColor, compositeMask, 80);

        // deproject depth image
        // pcl::PointCloud<pcl::PointXYZ>cloud;
        pcl::PointCloud<pcl::PointXYZRGB>cloud;
        for (unsigned int i = 0; i < corners.size(); i++)
        {
            auto u = corners[i].y;
            auto v = corners[i].x;
            if (inputDepth.at<ushort>(u,v) > minDepth && inputDepth.at<ushort>(u,v) < maxDepth)  // ignore min max depth values and pixels outside of mask
            {
                // pcl::PointXYZ point;
                pcl::PointXYZRGB point;
                float z = (inputDepth.at<ushort>(u,v))/1000.0f;   // convert depth from mm to m
                float x = (camMatInv.at<float>(0,0)*(v) + camMatInv.at<float>(0,2))*z; 
                float y = (camMatInv.at<float>(1,1)*(u) + camMatInv.at<float>(1,2))*z;
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
        // for (unsigned int u = 0; u < inputDepth.rows; u++)
        // {
        //     for (unsigned int v = 0; v < inputDepth.cols; v++)
        //     {

        //         if (compositeMask.at<uchar>(u,v) > 0 && inputDepth.at<ushort>(u,v) > minDepth && inputDepth.at<ushort>(u,v) < maxDepth)  // ignore min max depth values and pixels outside of mask
        //         {
        //             // pcl::PointXYZ point;
        //             pcl::PointXYZRGB point;
        //             float z = (inputDepth.at<ushort>(u,v))/1000.0f;   // convert depth from mm to m
        //             float x = (camMatInv.at<float>(0,0)*(v) + camMatInv.at<float>(0,2))*z; 
        //             float y = (camMatInv.at<float>(1,1)*(u) + camMatInv.at<float>(1,2))*z;
        //             // xyz is rotated to transform from camera frame coords to TF of camera
        //             point.x = z;
        //             point.y = -x;
        //             point.z = -y;

        //             std::uint32_t rgb = (static_cast<std::uint32_t>(inputColor.at<cv::Vec3b>(u,v)[0]) << 16 | 
        //                                  static_cast<std::uint32_t>(inputColor.at<cv::Vec3b>(u,v)[1]) << 8 | static_cast<std::uint32_t>(inputColor.at<cv::Vec3b>(u,v)[2]));

        //             point.rgb = *reinterpret_cast<float*>(&rgb);
        //             cloud.points.push_back(point);
        //         }
        //     }
        // }

        cloud.header.frame_id = camFrameID;
        pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
        pub.publish(cloud);

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
