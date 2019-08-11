#include "simple_renderer_nodecore.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

SimpleRendererNode::SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    imagePublisher_ = it_.advertise("image_out", 1);
    imageSubscriber_ = it_.subscribe("image_in", 1, &SimpleRendererNode::imageCallback, this);

    int width, height;
    nh_.param<int>("width", width, 640);
    nh_.param<int>("height", height, 480);

    std::string vertexShader, fragmentShader;
    nh_.param<std::string>("vertex_shader", vertexShader, "");
    nh_.param<std::string>("fragment_shader", fragmentShader, "");

    renderer_ = std::make_unique<cgs::SimpleRenderer>(
        width, height, 
        vertexShader, fragmentShader
    );

    output_.create(height, width, CV_8UC3);
}

void SimpleRendererNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    const auto& image = cv_ptr->image;
    renderer_->render(output_, image);

    //TODO publish
}

void SimpleRendererNode::run()
{
    ros::spin();
}