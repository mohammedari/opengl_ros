#ifndef SIMPLE_RENDERER_NODECORE_H
#define SIMPLE_RENDERER_NODECORE_H

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <opengl_ros/opengl_rosConfig.h>

#include "simple_renderer.h"


namespace opengl_ros {

class SimpleRendererNode
{
    //Handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    //Publishers and Subscribers
    image_transport::Publisher imagePublisher_;
    image_transport::Subscriber imageSubscriber_;
    
    //Other members
    std::unique_ptr<cgs::SimpleRenderer> renderer_;
    cv::Mat output_;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void reconfigure_callback(opengl_ros::opengl_rosConfig &config, uint32_t level);
    float threshold_l_, svm_coef_a_, svm_coef_b_, svm_intercept_;


public:
    SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif