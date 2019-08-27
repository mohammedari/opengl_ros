#include "depth_image_projector_nodecore.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

DepthImageProjectorNode::DepthImageProjectorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    imagePublisher_  = it_.advertise("image_out", 1);
    imageSubscriber_ = it_.subscribe("image_in" , 1, &DepthImageProjectorNode::imageCallback, this);
    
    //TODO OcupancyGridをPublishするようにする

    int depthWidth, depthHeight;
    nh_.param<int>("depthWidth" , depthWidth , 640);
    nh_.param<int>("depthHeight", depthHeight, 360);

    int gridMapWidth, gridMapHeight;
    double gridMapResolution, gridMapLayerHeight;
    nh_.param<int>("gridMapWidth" , gridMapWidth , 1000);
    nh_.param<int>("gridMapHeight", gridMapHeight, 1000);
    nh_.param<double>("gridMapResolution", gridMapResolution, 0.01);
    nh_.param<double>("gridMapLayerHeight", gridMapLayerHeight, 1);

    std::string vertexShader, fragmentShader;
    nh_.param<std::string>("vertex_shader"  , vertexShader  , "");
    nh_.param<std::string>("fragment_shader", fragmentShader, "");

    projector_ = std::make_unique<cgs::DepthImageProjector>(
        depthWidth, depthHeight, 
        gridMapWidth, gridMapHeight, gridMapResolution, gridMapLayerHeight,
        vertexShader, fragmentShader
    );

    output_.create(gridMapHeight, gridMapWidth, CV_8SC1);
}

void DepthImageProjectorNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    const auto& image = cv_ptr->image;
    projector_->project(output_, image);

    //Publish
    //TODO change to publish occupancy grid
    cv_bridge::CvImage outImage;;
    outImage.header = cv_ptr->header;
    outImage.encoding = sensor_msgs::image_encodings::MONO8;
    outImage.image = output_;
    imagePublisher_.publish(outImage.toImageMsg());
}

void DepthImageProjectorNode::run()
{
    ros::spin();
}