#ifndef OBJECT_POSITION_EXTRACTOR_NODECORE_H
#define OBJECT_POSITION_EXTRACTOR_NODECORE_H

#include <array>
#include <memory>
#include <limits>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

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
    ros::Publisher objectArrayPublisher_;
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
    double object_separation_distance_;
    int min_pixel_count_for_detection_;

    void colorCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg);
    void getTransformMatrixArray(const tf::Transform& transform, std::array<float, 16>& matrix);
    bool updateDepthToColor();

public:
    struct ObjectCandidate
    {
        std::vector<Eigen::Vector3d> points;
        Eigen::Vector3d sum = {};

        void add(const Eigen::Vector3d& point)
        {
            points.push_back(point);
            sum += point;
        }

        Eigen::Vector3d mean() const
        {
            return sum / points.size();
        }

        int count() const
        {
            return points.size();
        }

        Eigen::Vector3d size() const 
        {
            double min_x = std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double min_z = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();
            double max_z = -std::numeric_limits<double>::infinity();
            for (const auto p : points)
            {
                if (p.x() < min_x) min_x = p.x();
                if (p.y() < min_y) min_y = p.y();
                if (p.z() < min_z) min_z = p.z();
                if (max_x < p.x()) max_x = p.x();
                if (max_y < p.y()) max_y = p.y();
                if (max_z < p.z()) max_z = p.z();
            }

            return Eigen::Vector3d(
                max_x - min_x, 
                max_y - min_y, 
                max_z - min_z 
            );
        }
    };

public:
    ObjectPositionExtractorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif
