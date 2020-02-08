#include "object_position_extractor_nodecore.h"

#include <limits>
#include <tuple>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cgs_nav_msgs/Object.h>
#include <cgs_nav_msgs/ObjectArray.h>

using namespace opengl_ros;

ObjectPositionExtractorNode::ObjectPositionExtractorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    imagePublisher_  = it_.advertise("image_out", 1);
    objectArrayPublisher_ = nh_.advertise<cgs_nav_msgs::ObjectArray>("objects", 5);
    //TODO synchronize two image topics
    colorSubscriber_ = it_.subscribeCamera("color_in" , 1, &ObjectPositionExtractorNode::colorCallback, this);
    depthSubscriber_ = it_.subscribeCamera("depth_in" , 1, &ObjectPositionExtractorNode::depthCallback, this);

    //Frame ids for tf which associates color frame with depth frame
    nh_.param<std::string>("color_frame_id", color_frame_id_, "d435_color_optical");
    nh_.param<std::string>("depth_frame_id", depth_frame_id_, "d435_depth_optical");
    nh_.param<double>("tf_wait_duration", tf_wait_duration_, 0.1);

    //Other parameters
    nh_.param<double>("object_separation_distance", object_separation_distance_, 2);
    nh_.param<int>("min_pixel_count_for_detection", min_pixel_count_for_detection_, 10);
    nh_.param<double>("sigma_coefficient_", sigma_coefficient_, 2);

    //OpenGL parameters
    int color_width, color_height;
    nh_.param<int>("color_width" , color_width , 640);
    nh_.param<int>("color_height", color_height, 360);
    int depth_width, depth_height;
    nh_.param<int>("depth_width" , depth_width , 640);
    nh_.param<int>("depth_height", depth_height, 360);
    int output_width, output_height;
    nh_.param<int>("output_width" , output_width , 320);
    nh_.param<int>("output_height", output_height, 180);

    double min_depth, max_depth;
    nh_.param<double>("min_depth", min_depth, 0.105);
    nh_.param<double>("max_depth", max_depth, 10);

    std::string vertex_shader, fragment_shader;
    nh_.param<std::string>("vertex_shader"   , vertex_shader  , "");
    nh_.param<std::string>("fragment_shader" , fragment_shader, "");

    extractor_ = std::make_unique<cgs::ObjectPositionExtractor>(
        color_width, color_height, 
        depth_width, depth_height, 
        output_width, output_height, 
        min_depth, max_depth, 
        vertex_shader, fragment_shader
    );

    float threshold_l, svm_coef_a, svm_coef_b, svm_intercept;
    nh_.param<float>("threshold_l"  , threshold_l  , 0);
    nh_.param<float>("svm_coef_a"   , svm_coef_a   , 0);
    nh_.param<float>("svm_coef_b"   , svm_coef_b   , 0);
    nh_.param<float>("svm_intercept", svm_intercept, 0);

    extractor_->updateExtractionParameter(threshold_l, svm_coef_a, svm_coef_b, svm_intercept);

    positionOut_.create(output_height, output_width, CV_32FC4);
    colorOut_.create(output_height, output_width, CV_8UC3);
}

void ObjectPositionExtractorNode::colorCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg)
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

    latestColorCameraInfo = *(cameraInfoMsg.get());
}

static std::tuple<int, double> findNearest(const std::vector<ObjectPositionExtractorNode::ObjectCandidate>& candidates, const Eigen::Vector3d point)
{
    ROS_ASSERT(candidates.size() > 0);

    int minIndex = 0;
    double minDistance = std::numeric_limits<double>::infinity();
    for (int i = 1; i < candidates.size(); ++i)
    {
        auto distance = (candidates[i].mean() - point).norm();
        if (distance < minDistance)
        {
            minIndex = i;
            minDistance = distance;
        }
    }

    return std::tie(minIndex, minDistance);
}

void ObjectPositionExtractorNode::depthCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg)
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

    // Update & check whether depthToColor is valid.
    bool is_conversion_valid = updateDepthToColor();
    if (!is_conversion_valid)
    {
        ROS_WARN_STREAM("extrinsics parameter not arrivied yet");
        return;
    }

    //Update projector parameters with camera info
    extractor_->updateProjectionMatrix(
        {static_cast<float>(latestColorCameraInfo.K[0]), static_cast<float>(latestColorCameraInfo.K[4])}, 
        {static_cast<float>(latestColorCameraInfo.K[2]), static_cast<float>(latestColorCameraInfo.K[5])}, 
        {static_cast<float>(cameraInfoMsg->K[0])           , static_cast<float>(cameraInfoMsg->K[4])}, 
        {static_cast<float>(cameraInfoMsg->K[2])           , static_cast<float>(cameraInfoMsg->K[5])}, 
        latestDepthToColor_
    );

    //Perform projection
    const auto& color = latestColorImagePtr->image;
    const auto& depth = cv_ptr->image;
    extractor_->extract(positionOut_, colorOut_, color, depth);

    //Publish debug image
    cv_bridge::CvImage outImage;;
    outImage.header = cv_ptr->header;
    outImage.encoding = "rgb8";
    outImage.image = colorOut_;
    imagePublisher_.publish(outImage.toImageMsg());

    //Clustering the detected pixels
    std::vector<ObjectPositionExtractorNode::ObjectCandidate> candidates;
    for (auto it = positionOut_.begin<cv::Vec4f>(); it != positionOut_.end<cv::Vec4f>(); ++it)
    {
        auto x = (*it)[0];
        auto y = (*it)[1];
        auto z = (*it)[2];
        
        //Not detected at this pixel
        if (x == 0 && y == 0 && z == 0)
            continue;

        Eigen::Vector3d point(x, y, z);

        //If this is the first point, just add it to the candidates
        if (candidates.size() == 0)
        {
            candidates.emplace_back();
            candidates[0].add(point);
            continue;
        }

        //Find a candidate nearest from the point
        int minIndex;
        double minDistance;
        std::tie(minIndex, minDistance) = findNearest(candidates, point);

        //If the candidate is distant from the point, create a new candidate
        if (object_separation_distance_ < minDistance)
        {
            candidates.emplace_back();
            candidates[candidates.size() - 1].add(point);
            continue;
        }

        //Add the point to the neareset candidate
        candidates[minIndex].add(point);
    }

    //Publish Object Array
    cgs_nav_msgs::ObjectArray objectArray;
    for (const auto& candidate : candidates)
    {
        auto sigma = candidate.variance();

        auto count = candidate.count(sigma_coefficient_ * sigma);
        if (count < min_pixel_count_for_detection_)
            continue;

        ROS_INFO_STREAM("detected object (" << count << "/" << candidate.count() << "px)");

        auto candidate_pose = candidate.mean(sigma_coefficient_ * sigma);
        auto candidate_size = candidate.size(sigma_coefficient_ * sigma);
        geometry_msgs::Pose pose;
        {
            pose.position.x = candidate_pose.x();
            pose.position.y = candidate_pose.y();
            pose.position.z = candidate_pose.z();
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;
        }
        geometry_msgs::Twist zero_twist;
        geometry_msgs::Vector3 size;
        {
            size.x = candidate_size.x();
            size.y = candidate_size.y();
            size.z = candidate_size.z();
        }

        cgs_nav_msgs::Object o;
        o.header.frame_id = depth_frame_id_;
        o.header.stamp = cv_ptr->header.stamp;
        o.child_frame_id = "object"; //TODO
        o.id = 0; //TODO
        o.pose = pose;
        o.twist = zero_twist;
        o.size = size;
        o.confidence = static_cast<double>(count) / candidate.count(); //set number of pixels in threshold as confidence

        objectArray.objects.push_back(o);
    }
    objectArray.header.frame_id = depth_frame_id_;
    objectArray.header.stamp = cv_ptr->header.stamp;
    objectArrayPublisher_.publish(objectArray);
}

void ObjectPositionExtractorNode::getTransformMatrixArray(const tf::Transform& transform, std::array<float, 16>& matrix)
{
    const tf::Matrix3x3& R = transform.getBasis();
    const tf::Vector3& t =  transform.getOrigin();

    matrix = {
        static_cast<float>(R[0][0]), static_cast<float>(R[1][0]), static_cast<float>(R[2][0]), 0.0,
        static_cast<float>(R[0][1]), static_cast<float>(R[1][1]), static_cast<float>(R[2][1]), 0.0,
        static_cast<float>(R[0][2]), static_cast<float>(R[1][2]), static_cast<float>(R[2][2]), 0.0,
        static_cast<float>(t[0]),    static_cast<float>(t[1]),    static_cast<float>(t[2]),    1.0,
    }; //column major order
}

bool ObjectPositionExtractorNode::updateDepthToColor()
{
    // Calculation is performed only once since depthToColor does not change.
    if (depthToColorArrived_)
    {
        return true;
    }

    try
    {
        if (tfListener_.waitForTransform(color_frame_id_, depth_frame_id_, ros::Time(0), ros::Duration(tf_wait_duration_)))
        {
            tf::StampedTransform transform;
            tfListener_.lookupTransform(color_frame_id_, depth_frame_id_, ros::Time(0), transform);

            getTransformMatrixArray(transform, latestDepthToColor_);

            depthToColorArrived_ = true;
        }
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("Transform Exception in updateDepthToColor(). %s", ex.what());
    }

    return depthToColorArrived_;
}

void ObjectPositionExtractorNode::run()
{
    ros::spin();
}
