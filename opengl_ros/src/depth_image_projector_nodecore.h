#ifndef DEPTH_IMAGE_PROJECTOR_NODECORE_H
#define DEPTH_IMAGE_PROJECTOR_NODECORE_H

#include <array>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

#include "depth_image_projector.h"

namespace opengl_ros {

class DepthImageProjectorNode
{
    //Handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    //tf
    std::string color_frame_id_;
    std::string depth_frame_id_;
    std::string map_frame_id_;
    tf::TransformListener tfListener_;
    double tf_wait_duration_;

    //Other parameters
    int gridMapWidth_, gridMapHeight_;
    double gridMapResolution_;

    //Publishers and Subscribers
    ros::Publisher mapPublisher_;
    image_transport::CameraSubscriber colorSubscriber_;
    image_transport::CameraSubscriber depthSubscriber_;
    ros::Subscriber depthToColorSubscriber_;
    
    //Other members
    std::unique_ptr<cgs::DepthImageProjector> projector_;
    cv::Mat output_;

    cv_bridge::CvImageConstPtr      latestColorImagePtr;
    sensor_msgs::CameraInfoConstPtr latestColorCameraInfoPtr;
    void colorCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg);
    bool depthToColorArrived_ = false;
    void getTransformMatrixArray(const tf::Transform& transform, std::array<float, 16>& matrix);
    std::array<float, 16> latestDepthToColor_;
    bool updateDepthToColor();

public:
    DepthImageProjectorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif
