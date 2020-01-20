#include "depth_image_projector_nodecore.h"

#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace opengl_ros;

DepthImageProjectorNode::DepthImageProjectorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    mapPublisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);

    //TODO synchronize two image topics
    colorSubscriber_ = it_.subscribeCamera("color_in" , 1, &DepthImageProjectorNode::colorCallback, this);
    depthSubscriber_ = it_.subscribeCamera("depth_in" , 1, &DepthImageProjectorNode::depthCallback, this);

    // frame ids for tf which associates color frame with depth frame
    nh_.param<std::string>("color_frame_id", color_frame_id_, "d435_color_optical");
    nh_.param<std::string>("depth_frame_id", depth_frame_id_, "d435_depth_optical");
    nh_.param<std::string>("map_frame_id", map_frame_id_, "d435_depth_link");
    nh_.param<double>("tf_wait_duration", tf_wait_duration_, 0.1);

    int colorWidth, colorHeight;
    nh_.param<int>("colorWidth" , colorWidth , 640);
    nh_.param<int>("colorHeight", colorHeight, 360);
    int depthWidth, depthHeight;
    nh_.param<int>("depthWidth" , depthWidth , 640);
    nh_.param<int>("depthHeight", depthHeight, 360);

    double gridMapLayerHeight, gridMapAccumulationWeight;
    nh_.param<int>("gridMapWidth" , gridMapWidth_ , 1000);
    nh_.param<int>("gridMapHeight", gridMapHeight_, 1000);
    nh_.param<double>("gridMapResolution", gridMapResolution_, 0.01);
    nh_.param<double>("gridMapLayerHeight", gridMapLayerHeight, 1);
    nh_.param<double>("gridMapAccumulationWeight", gridMapAccumulationWeight, 1);

    double minDepth, maxDepth, depthHitThreshold;
    int unknownDepthColor;
    nh_.param<double>("minDepth", minDepth, 0.105);
    nh_.param<double>("maxDepth", maxDepth, 10);
    nh_.param<double>("depthHitThreshold", depthHitThreshold, 0.95);
    nh_.param<int>("unknownDepthColor", unknownDepthColor, 255);

    std::string vertexShader, geometryShader, fragmentShader, vertexShaderScaling, fragmentShaderScaling;
    nh_.param<std::string>("vertex_shader"   , vertexShader  , "");
    nh_.param<std::string>("geometry_shader" , geometryShader, "");
    nh_.param<std::string>("fragment_shader" , fragmentShader, "");
    nh_.param<std::string>("vertex_shader_scaling" , vertexShaderScaling, "");
    nh_.param<std::string>("fragment_shader_scaling", fragmentShaderScaling, "");

    projector_ = std::make_unique<cgs::DepthImageProjector>(
        colorWidth, colorHeight, 
        depthWidth, depthHeight, 
        gridMapWidth_, gridMapHeight_, gridMapResolution_, 
        gridMapLayerHeight, gridMapAccumulationWeight,
        minDepth, maxDepth, depthHitThreshold, unknownDepthColor, 
        vertexShader, geometryShader, fragmentShader, 
        vertexShaderScaling, fragmentShaderScaling
    );

    float threshold_l, svm_coef_a, svm_coef_b, svm_intercept;
    nh_.param<float>("threshold_l"  , threshold_l  , 0);
    nh_.param<float>("svm_coef_a"   , svm_coef_a   , 0);
    nh_.param<float>("svm_coef_b"   , svm_coef_b   , 0);
    nh_.param<float>("svm_intercept", svm_intercept, 0);

    //TODO Remove entire color image process as color image is not used anymore
    //projector_->uniform("threshold_l"  , threshold_l);
    //projector_->uniform("svm_coef_a"   , svm_coef_a);
    //projector_->uniform("svm_coef_b"   , svm_coef_b);
    //projector_->uniform("svm_intercept", svm_intercept);

    output_.create(gridMapHeight_, gridMapWidth_, CV_8UC1);
}

void DepthImageProjectorNode::colorCallback(const sensor_msgs::Image::ConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg)
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

    latestColorCameraInfoPtr = cameraInfoMsg;
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

    //TODO Remove entire color image process as color image is not used anymore
    //if (!latestColorImagePtr)
    //{
    //    ROS_WARN_STREAM("color stream not ready");
    //    return;
    //}

    //TODO Remove entire color image process as color image is not used anymore
    //// Update & check whether depthToColor is valid.
    //bool is_conversion_valid = updateDepthToColor();
    //if (!is_conversion_valid)
    //{
    //    ROS_WARN_STREAM("extrinsics parameter not arrivied yet");
    //    return;
    //}

    //Update projector parameters with camera info
    projector_->updateProjectionMatrix(
        {static_cast<float>(latestColorCameraInfoPtr->K[0]), static_cast<float>(latestColorCameraInfoPtr->K[4])}, 
        {static_cast<float>(latestColorCameraInfoPtr->K[2]), static_cast<float>(latestColorCameraInfoPtr->K[5])}, 
        {static_cast<float>(cameraInfoMsg->K[0])           , static_cast<float>(cameraInfoMsg->K[4])}, 
        {static_cast<float>(cameraInfoMsg->K[2])           , static_cast<float>(cameraInfoMsg->K[5])}, 
        latestDepthToColor_
    );

    //Perform projection
    const auto& color = latestColorImagePtr->image;
    const auto& depth = cv_ptr->image;
    projector_->project(output_, color, depth);

    //Publish
    nav_msgs::MapMetaData mapInfo;
    mapInfo.resolution = gridMapResolution_;
    mapInfo.width = gridMapWidth_;
    mapInfo.height = gridMapHeight_;
    mapInfo.origin.position.x = gridMapWidth_  * gridMapResolution_;
    mapInfo.origin.position.y = gridMapHeight_ * gridMapResolution_ / 2;
    tf2::Quaternion orientation;
    orientation.setRPY( 0, M_PI, M_PI / 2 );
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

bool DepthImageProjectorNode::updateDepthToColor()
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

            const tf::Matrix3x3& R = transform.getBasis();
            const tf::Vector3& t =  transform.getOrigin();

            latestDepthToColor_ = {
                static_cast<float>(R[0][0]), static_cast<float>(R[1][0]), static_cast<float>(R[2][0]), 0.0,
                static_cast<float>(R[0][1]), static_cast<float>(R[1][1]), static_cast<float>(R[2][1]), 0.0,
                static_cast<float>(R[0][2]), static_cast<float>(R[1][2]), static_cast<float>(R[2][2]), 0.0,
                static_cast<float>(t[0]),    static_cast<float>(t[1]),    static_cast<float>(t[2]),    1.0,
            }; //column major order
            depthToColorArrived_ = true;
        }
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("Transform Exception in updateDepthToColor(). %s", ex.what());
    }

    return depthToColorArrived_;
}

void DepthImageProjectorNode::run()
{
    ros::spin();
}
