#include "segpc_yolov7/segment_pc_node.h"

PcSegmenter::PcSegmenter(ros::NodeHandle& _nh): nh{_nh}
{
    ros::param::get("/mask_topic", roiTopic);
    ros::param::get("/image_topic", imageTopic );
    ros::param::get("/camera_info_topic", camInfoTopic);
    ros::param::get("/publish_topic", pcTopic);
    roiAcquired = false;   // flag to prevent publishing before roi is acquired
    imageAcquired = false; // flag to prevent publishing before depth image is acquired
    cameraInfoSub = nh.subscribe(camInfoTopic,1, &PcSegmenter::cbCameraInfo, this);
    // PcSegmenter:pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(pcTopic, 1);
    PcSegmenter:pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(pcTopic, 5);
    colorAcquired = false;
    
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
        roiAcquired = true;
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
        std::cout << msg->encoding << "\n";
        //convert ROS sensor image msg to cv::Mat 
        depthImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image; // depth in mm as uint16
        // std::cout << std::to_string(depthImage.data->) << "\n";
        imageAcquired = true;
        lock.unlock(); // release the lock 
    }
}

void PcSegmenter::cbColorImage(const sensor_msgs::ImageConstPtr& msg) 
{
    if(lock.try_lock())
    { 
        //convert ROS sensor image msg to cv::Mat 
        colorImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3)->image; // depth in mm as uint16
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
    float fx = (float)msg.K[0];
    float fy = (float)msg.K[4];
    float ppx = (float)msg.K[2];
    float ppy = (float)msg.K[5];

    // calculate camera matrix inverse, store parameters as float64
    camMatInv = (cv::Mat_<float>(2,3) << 1/fx, 0, -ppx/fx, 0, 1/fy, -ppy/fy);    // assume skew is zero

    // unregister camera info subscriber
    cameraInfoSub.shutdown();

    // start bounding box and depth image subscriber threads
    roiSub = nh.subscribe(roiTopic, 1, &PcSegmenter::cbRoi, this); 
    PcSegmenter::imageSub = nh.subscribe(imageTopic, 1, &PcSegmenter::cbDepthImage, this);
    colorSub = nh.subscribe("/camera/color/image_raw", 1, &PcSegmenter::cbColorImage, this);

}

/*
callback to publish pointcloud using roi, depthImage, and cameraInfo
acquires lock to read roi, depthImage, and cameraInfo
call to acquire lock is blocking
*/
void PcSegmenter::publishPc()
{
    // check if roi and image have been acquired first
    if (roiAcquired == true && imageAcquired == true && colorAcquired == true) 
    {
        std::cout << "Publishing\n";
        // read newest data
        lock.lock();
        std::cout << "Test1\n";

        cv::Mat inputImage = depthImage.clone();
        std::vector<yolov7_ros::ObjectData> objDataList_func = objDataList;
        cv::Mat inputColor = colorImage.clone();
        std::cout << "Test2\n";

        // iterate through all detected objects
        cv::Mat compositeMask = cv_bridge::toCvCopy(objDataList_func[0].mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;
        for (unsigned int i = 1; i < objDataList_func.size(); i++) 
        {
            //bitwise_or to combine masks
            cv::Mat tempCompositeMask = compositeMask.clone();
            cv::Mat currMask = cv_bridge::toCvCopy(objDataList_func[i].mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;
            cv::bitwise_or(tempCompositeMask, currMask, compositeMask);
        }

        // deproject depth image
        // pcl::PointCloud<pcl::PointXYZ>cloud;
        pcl::PointCloud<pcl::PointXYZRGB>cloud;
        for (unsigned int u = 0; u < compositeMask.rows; u++)
        {
            for (unsigned int v = 0; v < compositeMask.cols; v++)
            {
                if (compositeMask.at<uint8_t>(u,v) > 0 && inputImage.at<uint16_t>(u,v) > 500 && inputImage.at<uint16_t>(u,v) < 3000)  // ignore min max depth values and pixels outside of mask
                {
                    // pcl::PointXYZ point;
                    pcl::PointXYZRGB point;
                    // float z = (float)inputImage.at<uint16_t>(u,v);   // convert depth from mm to m
         
                    float z = (float)inputImage.at<uint16_t>(u,v)/1000;   // convert depth from mm to m
                    // if ( z < .5)
                    // {
                    //     std::cout << std::to_string(z) << " === " << std::to_string(inputImage.at<uint16_t>(u,v)) << "\n";
                    // }
                    float x = (camMatInv.at<float>(0,0)*((float)v) + camMatInv.at<float>(0,2))*z; 
                    float y = (camMatInv.at<float>(1,1)*((float)u) + camMatInv.at<float>(1,2))*z;
                    // xyz is rotated to transform from camera frame coords to TF of camera
                    point.x = z;
                    point.y = -x;
                    point.z = -y;
                    std::cout << std::to_string(point.x) << " === " << std::to_string(inputImage.at<uint16_t>(u,v)) << "\n";

                    std::uint32_t rgb = (static_cast<std::uint32_t>(inputColor.at<cv::Vec3b>(u,v)[0]) << 16 | 
                                         static_cast<std::uint32_t>(inputColor.at<cv::Vec3b>(u,v)[1]) << 8 | static_cast<std::uint32_t>(inputColor.at<cv::Vec3b>(u,v)[2]));
                    point.rgb = *reinterpret_cast<float*>(&rgb);
                    cloud.points.push_back(point); 
                }
            }
        }
        lock.unlock();

        cloud.header.frame_id = camFrameID;
        pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
        pub.publish(cloud);
        std::cout << "Test3\n";

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
