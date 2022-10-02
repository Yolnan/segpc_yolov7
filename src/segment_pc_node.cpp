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
    // PcSegmenter:pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(PcSegmenter::pcTopic, 1);
    image_transport::ImageTransport it(PcSegmenter::nh);
    PcSegmenter::pub = it.advertise(PcSegmenter::pcTopic, 1);
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

/*
callback to get camera intrinsics and form inverse of camera matrix
params: sensor_msgs/camera_info
*/
void PcSegmenter::cbCameraInfo(const sensor_msgs::CameraInfo& msg) 
{
    // extract camera instrinsics parameters
    PcSegmenter::camFrameID = msg.header.frame_id;
    double fx = msg.K[0];
    double fy = msg.K[4];
    double ppx = msg.K[2];
    double ppy = msg.K[5];

    // calculate camera matrix inverse, store parameters as float64
    PcSegmenter::camMatInv = (cv::Mat_<double>(2,3) << 1/fx, 0, -ppx*fy/(fx*fy), 0, 1/fy, -ppy/fy);    // assume skew is zero

    // unregister camera info subscriber
    PcSegmenter::cameraInfoSub.shutdown();

    // start bounding box and depth image subscriber threads
    PcSegmenter::roiSub = PcSegmenter::nh.subscribe(PcSegmenter::roiTopic, 1, &PcSegmenter::cbRoi, this); 
    PcSegmenter::imageSub = PcSegmenter::nh.subscribe(PcSegmenter::imageTopic, 1, &PcSegmenter::cbDepthImage, this);

}

/*
callback to publish pointcloud using roi, depthImage, and cameraInfo
acquires lock to read roi, depthImage, and cameraInfo
call to acquire lock is blocking
*/
void PcSegmenter::publishPc()
{
    // check if roi and image have been acquired first
    if (PcSegmenter::roiAcquired == true && PcSegmenter::imageAcquired == true) 
    {
        
        // read newest data
        PcSegmenter::lock.lock();
        cv::Mat inputImage = PcSegmenter::depthImage.clone();
        std::vector<yolov7_ros::ObjectData> objDataList = PcSegmenter::objDataList;
        PcSegmenter::lock.unlock();

        // iterate through all detected objects
        cv::Mat outputImage = inputImage.clone();
        // cv::Mat compositeMask = cv::Mat::zeros(inputImage.size(), inputImage.type());  // what data type is input mask? 8UC1?
        cv::Mat compositeMask = cv_bridge::toCvCopy(objDataList[0].mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;
        for (unsigned int i = 1; i < objDataList.size(); i++) 
        {
            //bitwise_or to combine masks
            cv::Mat tempCompositeMask = compositeMask.clone();
            cv::Mat currMask = cv_bridge::toCvCopy(objDataList[i].mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;
            cv::bitwise_or(tempCompositeMask, currMask, compositeMask);


        }

        // apply mask
        inputImage.copyTo(outputImage, compositeMask);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, outputImage).toImageMsg();
        PcSegmenter::pub.publish(msg);

        // // deproject depth image
        // pcl::PointCloud<pcl::PointXYZ>cloud;
        // cloud.header.frame_id = PcSegmenter::camFrameID;
        // for (unsigned int u = 0; u < outputImage.rows; u++)
        // {
        //     for (unsigned int v = 0; v < outputImage.cols; v++)
        //     {
        //         if (outputImage.at<unsigned int>(u,v) > 0)  // ignore depth values of 0
        //         {
        //             double z = outputImage.at<unsigned int>(u,v);
        //             double x = (PcSegmenter::camMatInv.at<unsigned int>(0,0)*v + PcSegmenter::camMatInv.at<unsigned int>(0,2))*z; 
        //             double y = (PcSegmenter::camMatInv.at<unsigned int>(1,1)*u + PcSegmenter::camMatInv.at<unsigned int>(1,2))*z; 
        //             cloud.points.push_back(pcl::PointXYZ(z,-x,-y)); // xyz is rotated to transform from camera frame coords to TF of camera
        //         }
        //     }
        // }
        // pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
        // PcSegmenter::pub.publish(cloud);
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segment_pc_node");
    ros::NodeHandle nh;
    PcSegmenter segmentPcNode(nh);
    ros::Rate loopRate(10);
    while (ros::ok())
    {
        segmentPcNode.publishPc();
        ros::spinOnce();
    }
    return 0;
}
