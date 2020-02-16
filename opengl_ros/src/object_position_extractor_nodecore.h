#ifndef OBJECT_POSITION_EXTRACTOR_NODECORE_H
#define OBJECT_POSITION_EXTRACTOR_NODECORE_H

#include <array>
#include <memory>
#include <limits>
#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include "object_position_extractor.h"

#include "distance_matrix.h"

namespace opengl_ros {

class ObjectPositionExtractorNode
{
public:
    struct ObjectCandidate
    {
        uint64_t id = 0; 
        std::deque<tf::Vector3> points;
        tf::Vector3 sum = tf::Vector3{0, 0, 0};
        int total_number_of_detected_pixels = 0;
        ros::Time last_detected_time = ros::Time{0, 0};

        ObjectCandidate(uint64_t id) : id(id) {};

        void add(const tf::Vector3& point, int accumulated_pixel_count, int object_candidate_max_storing_points, const ros::Time& detected_time)
        {
            points.push_back(point);
            sum += point;
            total_number_of_detected_pixels += accumulated_pixel_count;
            last_detected_time = detected_time;

            if(points.size() > object_candidate_max_storing_points)
            {
                sum -= points.front();
                points.pop_front();
            }
        }

        tf::Vector3 mean() const
        {
            return sum / points.size();
        }

        bool mean(double threshold, tf::Vector3& result) const
        {
            auto m = mean();

            int count = 0;
            tf::Vector3 sum_in_variance = {};
            for (const auto p : points)
                if ((p - m).length() < threshold)
                {
                    sum_in_variance += p;
                    ++count;
                }

            if (count == 0)
                return false;

            result = sum_in_variance / count;
            return true;
        }

        double variance() const
        {
            auto m = mean();
            double squared_diff_sum = 0;
            for (const auto p : points)
            {
                squared_diff_sum += (p - m).length2();
            }

            return sqrt(squared_diff_sum / points.size());
        }

        bool size(double threshold, tf::Vector3& result) const 
        {
            auto m = mean();
            double min_x = std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double min_z = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();
            double max_z = -std::numeric_limits<double>::infinity();
            for (const auto p : points)
            {
                if (threshold < (p - m).length())
                    continue;

                if (p.x() < min_x) min_x = p.x();
                if (p.y() < min_y) min_y = p.y();
                if (p.z() < min_z) min_z = p.z();
                if (max_x < p.x()) max_x = p.x();
                if (max_y < p.y()) max_y = p.y();
                if (max_z < p.z()) max_z = p.z();
            }

            if (std::isinf(min_x))
                return false;

            result = tf::Vector3(
                max_x - min_x, 
                max_y - min_y, 
                max_z - min_z 
            );
            return true;
        }
    };

private:
    //Handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    //tf
    std::string color_frame_id_;
    std::string depth_frame_id_;
    std::string fixed_frame_id_;
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
    cv_bridge::CvImageConstPtr latestColorImagePtr;
    sensor_msgs::CameraInfo    latestColorCameraInfo;
    double object_separation_distance_;
    int target_pixel_count_threshold_;
    double sigma_coefficient_;
    double object_size_min_x_, object_size_max_x_;
    double object_size_min_y_, object_size_max_y_;
    double object_candidate_lifetime_;
    int object_candidate_max_storing_points_;
    std::list<ObjectCandidate> object_candidates_;
    uint64_t next_id_ = 0;

    int target_pixel_count_max_;
    std::unique_ptr<DistanceMatrix<float>> distance_matrix_;

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
