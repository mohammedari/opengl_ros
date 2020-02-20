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
    nh_.param<std::string>("fixed_frame_id", fixed_frame_id_, "map");
    nh_.param<double>("tf_wait_duration", tf_wait_duration_, 0.1);

    //Other parameters
    nh_.param<double>("object_separation_distance", object_separation_distance_, 2);
    nh_.param<int>("target_pixel_count_threshold", target_pixel_count_threshold_, 10);
    nh_.param<int>("target_pixel_count_max", target_pixel_count_max_, 1000);
    nh_.param<double>("sigma_coefficient_", sigma_coefficient_, 2);
    nh_.param<double>("object_size_min_x", object_size_min_x_, 0.1);
    nh_.param<double>("object_size_max_x", object_size_max_x_, 0.5);
    nh_.param<double>("object_size_min_y", object_size_min_y_, 0.1);
    nh_.param<double>("object_size_max_y", object_size_max_y_, 0.5);
    nh_.param<double>("object_candidate_lifetime", object_candidate_lifetime_, 0.5);

    //Preparation for DBSCAN
    double dbscan_epsilon;
    int dbscan_min_points;
    nh_.param<double>("dbscan_epsilon", dbscan_epsilon, 1.0);
    nh_.param<int>("dbscan_min_points", dbscan_min_points, 5);
    dbscan_ = std::make_unique<DBSCAN>(dbscan_epsilon, dbscan_min_points);
    distance_matrix_ = std::make_unique<DistanceMatrix<float>>(target_pixel_count_max_);

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

template<class CANDIDATE_LIST>
static std::tuple<ObjectPositionExtractorNode::ObjectCandidate*, double> findNearest(
    CANDIDATE_LIST& candidates, const tf::Vector3& point)
{
    ROS_ASSERT(candidates.size() > 0);

    ObjectPositionExtractorNode::ObjectCandidate* minElement;
    double minDistance = std::numeric_limits<double>::infinity();
    for (auto& candidate : candidates)
    {
        auto distance = (candidate.mean() - point).length();
        if (distance < minDistance)
        {
            minElement = &candidate;
            minDistance = distance;
        }
    }

    return std::tie(minElement, minDistance);
}

static void mergeCandidates(
    ObjectPositionExtractorNode::ObjectCandidate& candidate_world, 
    const ObjectPositionExtractorNode::ObjectCandidate& candidate_local, 
    int target_pixel_count_max,
    const tf::Transform& transform)
{
        //Convert the point to fixed_frame and add the points
        for (const auto& point : candidate_local.points)
            candidate_world.add(transform * point, 0, target_pixel_count_max, ros::Time());

        candidate_world.total_number_of_detected_pixels += candidate_local.total_number_of_detected_pixels;
        candidate_world.last_detected_time = candidate_local.last_detected_time;
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
        {static_cast<float>(cameraInfoMsg->K[0])       , static_cast<float>(cameraInfoMsg->K[4])}, 
        {static_cast<float>(cameraInfoMsg->K[2])       , static_cast<float>(cameraInfoMsg->K[5])}, 
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

    //Convert accumulated coordinates in an image to 3D points
    std::vector<tf::Vector3> points;
    std::vector<int> accumulated_pixel_counts;
    for (auto it = positionOut_.begin<cv::Vec4f>(); it != positionOut_.end<cv::Vec4f>(); ++it)
    {
        //Use only first points less than maximum capacity
        //to ensure finishing the process in an acceptable time
        if (target_pixel_count_max_ <= points.size())
        {
            ROS_WARN_STREAM("Target pixel count exceeds 'target_pixel_count_max'. First " << target_pixel_count_max_ << " pixels will be used for identify targets.");
            break;
        }

        auto x = (*it)[0]; //
        auto y = (*it)[1]; //
        auto z = (*it)[2]; //accumulated coordinate value
        auto w = (*it)[3]; //number of accumulated point 
        
        //Not detected at this pixel
        if (w == 0)
            continue;

        points.emplace_back(x / w, y / w, z / w);
        accumulated_pixel_counts.push_back(static_cast<int>(std::round(w)));
    }

    //Clustering the detected 3D points with DBSCAN
    std::vector<ObjectPositionExtractorNode::ObjectCandidate> candidates_in_local_space;
    {
        const auto distance_calculator = [](const tf::Vector3& x, const tf::Vector3& y){ return (x - y).length(); };
        distance_matrix_->update<std::vector<tf::Vector3>, tf::Vector3>(points, distance_calculator);
        auto clusters = dbscan_->cluster(*distance_matrix_.get());

        candidates_in_local_space.resize(clusters.size());
        for (int i = 0; i < clusters.size(); ++i)
            for (auto index : clusters[i])
                candidates_in_local_space[i].add(points[index], accumulated_pixel_counts[index], target_pixel_count_max_, cv_ptr->header.stamp);
    }

    //Retrieve tf for fixed_frame;
    tf::StampedTransform transform_to_fixed_frame;
    try
    {
        if (tfListener_.waitForTransform(fixed_frame_id_, depth_frame_id_,
              cv_ptr->header.stamp, ros::Duration(tf_wait_duration_)))
        {
            tfListener_.lookupTransform(fixed_frame_id_, depth_frame_id_,
                cv_ptr->header.stamp, transform_to_fixed_frame);
        } else {
            ROS_WARN_STREAM("TF wait transform timeout in depthCallback().");
            return;
        }
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_STREAM("Transform Exception occured in depthCallback(). " << ex.what());
        return;
    }

    //Merge candidates into world space candidates
    for (const auto& candidate : candidates_in_local_space)
    {
        auto sigma = candidate.variance();
        tf::Vector3 candidate_mean, candidate_size; 
        if (!candidate.mean(sigma_coefficient_ * sigma, candidate_mean) || !candidate.size(sigma_coefficient_ * sigma, candidate_size))
        {
            ROS_DEBUG_STREAM("Too small sigma coefficient in calculating mean/size of local object candidate.");
            continue;
        }

        //If the size of the object along the camera direction is too small or too large, skip it
        if (candidate_size.x() < object_size_min_x_ || object_size_max_x_ < candidate_size.x() || 
            candidate_size.y() < object_size_min_y_ || object_size_max_y_ < candidate_size.y())
            continue;

        ROS_INFO_STREAM("detected object candidate (" << candidate.total_number_of_detected_pixels << "px)");

        //If this is the first candidate, just add it to the candidates
        if (object_candidates_.size() == 0)
        {
            object_candidates_.emplace_back(next_id_++);
            mergeCandidates(object_candidates_.back(), candidate, target_pixel_count_max_, transform_to_fixed_frame);
            continue;
        }

        //Calculate mean of the candidate in world space
        candidate_mean = transform_to_fixed_frame * candidate_mean;

        //Find a candidate nearest from the mean of the local candidate
        ObjectPositionExtractorNode::ObjectCandidate* minElement;
        double minDistance;
        std::tie(minElement, minDistance) = findNearest(object_candidates_, candidate_mean);

        //If the candidate is distant from the point, create a new candidate
        if (object_separation_distance_ < minDistance)
        {
            object_candidates_.emplace_back(next_id_++);
            mergeCandidates(object_candidates_.back(), candidate, target_pixel_count_max_, transform_to_fixed_frame);
            continue;
        }

        //Merge the candidate to the neareset candidate
        mergeCandidates(*minElement, candidate, target_pixel_count_max_, transform_to_fixed_frame);
    }

    //Remove out-dated candidates
    auto lifetime = object_candidate_lifetime_;
    auto stamp = cv_ptr->header.stamp;
    object_candidates_.remove_if([lifetime, stamp](const auto& candidate) {
        return ros::Duration(lifetime) < stamp - candidate.last_detected_time;
    });

    //Publish Object Array
    cgs_nav_msgs::ObjectArray objectArray;
    for (const auto& candidate : object_candidates_)
    {
        //If number of pixel is lower than threshold, skip it
        if (candidate.total_number_of_detected_pixels < target_pixel_count_threshold_)
            continue;

        auto sigma = candidate.variance();
        tf::Vector3 candidate_mean, candidate_size;
        if (!candidate.mean(sigma_coefficient_ * sigma, candidate_mean) || !candidate.size(sigma_coefficient_ * sigma, candidate_size))
        {
            ROS_DEBUG_STREAM("Too small sigma coefficient in calculating mean/size of world object candidate.");
            continue;
        }

        geometry_msgs::Pose pose;
        {
            pose.position.x = candidate_mean.x();
            pose.position.y = candidate_mean.y();
            pose.position.z = candidate_mean.z();
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;
        }
        geometry_msgs::Twist zero_twist;
        geometry_msgs::Vector3 size;
        {
            //tweek the size to avoid rviz axis aligned box assertion
            size.x = std::max(0.001, candidate_size.x());
            size.y = std::max(0.001, candidate_size.y());
            size.z = std::max(0.001, candidate_size.z());
        }

        cgs_nav_msgs::Object o;
        o.header.frame_id = fixed_frame_id_;
        o.header.stamp = cv_ptr->header.stamp;
        o.child_frame_id = "object"; //TODO
        o.id = candidate.id; 
        o.pose = pose;
        o.twist = zero_twist;
        o.size = size;
        o.confidence = 1.0; //TODO

        objectArray.objects.push_back(o);
    }
    objectArray.header.frame_id = fixed_frame_id_;
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
        } else {
            ROS_WARN_STREAM("TF wait transform timeout in depthCallback().");
            return false;
        }
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_STREAM("Transform Exception in updateDepthToColor(). " << ex.what());
    }

    return depthToColorArrived_;
}

void ObjectPositionExtractorNode::run()
{
    ros::spin();
}
