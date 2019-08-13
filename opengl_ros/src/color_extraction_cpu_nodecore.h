#ifndef COLOR_EXtRACTION_CPU_NODECORE_H
#define COLOR_EXtRACTION_CPU_NODECORE_H

#include <memory>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

namespace opengl_ros {

class ColorExtractionCpuNode
{
    //Handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    //Publishers and Subscribers
    image_transport::Publisher imagePublisher_;
    image_transport::Subscriber imageSubscriber_;

    //Parameters
    float threshold_l_;
    float svm_coef_a_;
    float svm_coef_b_;
    float svm_intercept_;

    cv::Mat lab_;
    cv::Mat output_;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

public:
    ColorExtractionCpuNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif