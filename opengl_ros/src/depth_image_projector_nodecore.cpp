#include "depth_image_projector_nodecore.h"

#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace opengl_ros;

DepthImageProjectorNode::DepthImageProjectorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    mapPublisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
    depthSubscriber_ = it_.subscribeCamera("depth_in" , 1, &DepthImageProjectorNode::depthCallback, this);

    nh_.param<std::string>("depth_frame_id", depth_frame_id_, "d435_depth_optical");
    nh_.param<std::string>("map_frame_id", map_frame_id_, "base_link_hor");
    nh_.param<double>("tf_wait_duration", tf_wait_duration_, 0.1);

    int depthWidth, depthHeight;
    nh_.param<int>("depth_width" , depthWidth , 640);
    nh_.param<int>("depth_height", depthHeight, 360);

    double gridMapLayerHeight, gridMapAccumulationWeight;
    nh_.param<int>("grid_map_width" , gridMapWidth_ , 1000);
    nh_.param<int>("grid_map_height", gridMapHeight_, 1000);
    nh_.param<double>("grid_map_resolution", gridMapResolution_, 0.01);
    nh_.param<double>("grid_map_layer_height", gridMapLayerHeight, 1);
    nh_.param<double>("grid_map_accumulation_weight", gridMapAccumulationWeight, 1);

    double minDepth, maxDepth, depthHitThreshold;
    int unknownDepthColor;
    nh_.param<double>("min_depth", minDepth, 0.105);
    nh_.param<double>("max_depth", maxDepth, 10);
    nh_.param<double>("depth_hit_threshold", depthHitThreshold, 0.95);
    nh_.param<int>("unknown_depth_color", unknownDepthColor, 255);

    std::string vertexShader, geometryShader, fragmentShader, vertexShaderScaling, fragmentShaderScaling;
    nh_.param<std::string>("vertex_shader"   , vertexShader  , "");
    nh_.param<std::string>("geometry_shader" , geometryShader, "");
    nh_.param<std::string>("fragment_shader" , fragmentShader, "");
    nh_.param<std::string>("vertex_shader_scaling" , vertexShaderScaling, "");
    nh_.param<std::string>("fragment_shader_scaling", fragmentShaderScaling, "");

    projector_ = std::make_unique<cgs::DepthImageProjector>(
        depthWidth, depthHeight, 
        gridMapWidth_, gridMapHeight_, gridMapResolution_, 
        gridMapLayerHeight, gridMapAccumulationWeight,
        minDepth, maxDepth, depthHitThreshold, unknownDepthColor, 
        vertexShader, geometryShader, fragmentShader, 
        vertexShaderScaling, fragmentShaderScaling
    );

    output_.create(gridMapHeight_, gridMapWidth_, CV_8UC1);
}

void DepthImageProjectorNode::depthCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg)
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
    
    std::array<float, 16> depthToMap;
    try
    {
        if (tfListener_.waitForTransform(map_frame_id_, cameraInfoMsg->header.frame_id,
              cameraInfoMsg->header.stamp, ros::Duration(tf_wait_duration_)))
        {
            tf::StampedTransform transform;
            tfListener_.lookupTransform(map_frame_id_, cameraInfoMsg->header.frame_id,
                cameraInfoMsg->header.stamp, transform);
            getTransformMatrixArray(transform, depthToMap);
        }
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("Transform Exception in depthCallback(). %s", ex.what());
        return;
    }

    //Update projector parameters with camera info
    projector_->updateProjectionMatrix(
        {static_cast<float>(cameraInfoMsg->K[0])           , static_cast<float>(cameraInfoMsg->K[4])}, 
        {static_cast<float>(cameraInfoMsg->K[2])           , static_cast<float>(cameraInfoMsg->K[5])}, 
        depthToMap
    );

    //Perform projection
    projector_->project(output_, cv_ptr->image);

    //Publish
    nav_msgs::MapMetaData mapInfo;
    mapInfo.resolution = gridMapResolution_;
    mapInfo.width = gridMapWidth_;
    mapInfo.height = gridMapHeight_;
    mapInfo.origin.position.x = -gridMapHeight_ * gridMapResolution_ / 2;
    mapInfo.origin.position.y = -gridMapWidth_ * gridMapResolution_ / 2;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, 0.0);
    mapInfo.origin.orientation = tf2::toMsg(orientation);

    nav_msgs::OccupancyGrid grid;
    grid.header.seq      = cv_ptr->header.seq;
    grid.header.stamp    = cv_ptr->header.stamp;
    grid.header.frame_id = map_frame_id_;
    grid.info = mapInfo;

    auto data = reinterpret_cast<int8_t*>(output_.data);
    auto size = gridMapWidth_ * gridMapHeight_;
    grid.data.reserve(size);
    std::copy(data, data + size, std::back_inserter(grid.data));
    mapPublisher_.publish(grid);

}

void DepthImageProjectorNode::getTransformMatrixArray(const tf::Transform& transform, std::array<float, 16>& matrix)
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

void DepthImageProjectorNode::run()
{
    ros::spin();
}
