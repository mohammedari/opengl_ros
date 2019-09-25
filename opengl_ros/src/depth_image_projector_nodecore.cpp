#include "depth_image_projector_nodecore.h"

#include <opencv2/opencv.hpp>

using namespace opengl_ros;

DepthImageProjectorNode::DepthImageProjectorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    imagePublisher_  = it_.advertise("image_out", 1);

    //TODO synchronize two image topics
    colorSubscriber_ = it_.subscribeCamera("color_in" , 1, &DepthImageProjectorNode::colorCallback, this);
    depthSubscriber_ = it_.subscribeCamera("depth_in" , 1, &DepthImageProjectorNode::depthCallback, this);

    int colorWidth, colorHeight;
    nh_.param<int>("colorWidth" , colorWidth , 640);
    nh_.param<int>("colorHeight", colorHeight, 360);
    int depthWidth, depthHeight;
    nh_.param<int>("depthWidth" , depthWidth , 640);
    nh_.param<int>("depthHeight", depthHeight, 360);

    int gridMapWidth, gridMapHeight;
    double gridMapResolution, gridMapLayerHeight, gridMapAccumulationWeight;
    nh_.param<int>("gridMapWidth" , gridMapWidth , 1000);
    nh_.param<int>("gridMapHeight", gridMapHeight, 1000);
    nh_.param<double>("gridMapResolution", gridMapResolution, 0.01);
    nh_.param<double>("gridMapLayerHeight", gridMapLayerHeight, 1);
    nh_.param<double>("gridMapAccumulationWeight", gridMapAccumulationWeight, 1);

    std::string vertexShader, fragmentShader;
    nh_.param<std::string>("vertex_shader"  , vertexShader  , "");
    nh_.param<std::string>("fragment_shader", fragmentShader, "");

    projector_ = std::make_unique<cgs::DepthImageProjector>(
        colorWidth, colorHeight, 
        depthWidth, depthHeight, 
        gridMapWidth, gridMapHeight, gridMapResolution, gridMapLayerHeight,
        gridMapAccumulationWeight,
        vertexShader, fragmentShader
    );

    output_.create(gridMapHeight, gridMapWidth, CV_8UC3);
}

void DepthImageProjectorNode::colorCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg)
{
    try
    {
        latestColorImagePtr = cv_bridge::toCvShare(imageMsg, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    latestColorCameraInfoPtr = cameraInfoMsg;
}

void DepthImageProjectorNode::depthCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(imageMsg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    if (!latestColorImagePtr)
    {
        ROS_WARN_STREAM("color stream not ready");
        return;
    }

    //TODO update projector parameters with camera info

    const auto& color = latestColorImagePtr->image;
    const auto& depth = cv_ptr->image;
    projector_->project(output_, color, depth);

    //Publish
    cv_bridge::CvImage outImage;;
    outImage.header = cv_ptr->header;
    outImage.encoding = sensor_msgs::image_encodings::RGB8;
    outImage.image = output_;
    imagePublisher_.publish(outImage.toImageMsg());
}

void DepthImageProjectorNode::run()
{
    ros::spin();
}