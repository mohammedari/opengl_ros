#ifndef OBJECT_POSITION_EXTRACTOR_NODECORE_H
#define OBJECT_POSITION_EXTRACTOR_NODECORE_H

#include <array>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include "object_position_extractor.h"

namespace opengl_ros {

class ObjectPositionExtractorNode
{
    //Handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    //tf
    std::string color_frame_id_;
    std::string depth_frame_id_;
    tf::TransformListener tfListener_;
    double tf_wait_duration_;

    //Publishers and Subscribers
    image_transport::Publisher imagePublisher_;
    image_transport::CameraSubscriber colorSubscriber_;
    image_transport::CameraSubscriber depthSubscriber_;
    ros::Subscriber depthToColorSubscriber_;
    
    //Other members
    std::unique_ptr<cgs::ObjectPositionExtractor> extractor_;
    cv::Mat positionOut_, colorOut_;
    bool depthToColorArrived_ = false;
    std::array<float, 16> latestDepthToColor_;
    cv_bridge::CvImageConstPtr      latestColorImagePtr;
    sensor_msgs::CameraInfoConstPtr latestColorCameraInfoPtr;

    void colorCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg);
    void getTransformMatrixArray(const tf::Transform& transform, std::array<float, 16>& matrix);
    bool updateDepthToColor();

public:
    ObjectPositionExtractorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif
