#include "depth_image_projector_nodecore.h"

#include <opencv2/opencv.hpp>

using namespace opengl_ros;

DepthImageProjectorNode::DepthImageProjectorNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    imagePublisher_  = it_.advertise("image_out", 1);

    //TODO synchronize two image topics
    colorSubscriber_ = it_.subscribeCamera("color_in" , 1, &DepthImageProjectorNode::colorCallback, this);
    depthSubscriber_ = it_.subscribeCamera("depth_in" , 1, &DepthImageProjectorNode::depthCallback, this);

    //TODO change to use TF and remove dependency on realsense2_camera package
    depthToColorSubscriber_ = nh_.subscribe<realsense2_camera::Extrinsics>("depth_to_color" , 1, &DepthImageProjectorNode::depthToColorCallback, this);

    int colorWidth, colorHeight;
    nh_.param<int>("colorWidth" , colorWidth , 640);
    nh_.param<int>("colorHeight", colorHeight, 360);
    int depthWidth, depthHeight;
    nh_.param<int>("depthWidth" , depthWidth , 640);
    nh_.param<int>("depthHeight", depthHeight, 360);

    int gridMapWidth, gridMapHeight;
    double gridMapResolution, gridMapLayerHeight, gridMapAccumulationWeight;
    nh_.param<int>("gridMapWidth" , gridMapWidth , 1000);
    nh_.param<int>("gridMapHeight", gridMapHeight, 1000);
    nh_.param<double>("gridMapResolution", gridMapResolution, 0.01);
    nh_.param<double>("gridMapLayerHeight", gridMapLayerHeight, 1);
    nh_.param<double>("gridMapAccumulationWeight", gridMapAccumulationWeight, 1);

    std::string vertexShader, fragmentShader;
    nh_.param<std::string>("vertex_shader"   , vertexShader  , "");
    nh_.param<std::string>("geometry_shader" , geometryShader, "");
    nh_.param<std::string>("fragment_shader" , fragmentShader, "");

    projector_ = std::make_unique<cgs::DepthImageProjector>(
        colorWidth, colorHeight, 
        depthWidth, depthHeight, 
        gridMapWidth, gridMapHeight, gridMapResolution, gridMapLayerHeight,
        gridMapAccumulationWeight,
        vertexShader, geometryShader, fragmentShader
    );

    float threshold_l, svm_coef_a, svm_coef_b, svm_intercept;
    nh_.param<float>("threshold_l"  , threshold_l  , 0);
    nh_.param<float>("svm_coef_a"   , svm_coef_a   , 0);
    nh_.param<float>("svm_coef_b"   , svm_coef_b   , 0);
    nh_.param<float>("svm_intercept", svm_intercept, 0);

    projector_->uniform("threshold_l"  , threshold_l);
    projector_->uniform("svm_coef_a"   , svm_coef_a);
    projector_->uniform("svm_coef_b"   , svm_coef_b);
    projector_->uniform("svm_intercept", svm_intercept);

    output_.create(gridMapHeight, gridMapWidth, CV_8UC3);
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

    if (!latestColorImagePtr)
    {
        ROS_WARN_STREAM("color stream not ready");
        return;
    }

    if (!depthToColorArrived_)
    {
        ROS_WARN_STREAM("extrinsics parameter not arrivied yet");
        return;
    }

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
    cv_bridge::CvImage outImage;
    outImage.header.seq      = cv_ptr->header.seq;
    outImage.header.stamp    = cv_ptr->header.stamp;
    outImage.header.frame_id = "occupancy_grid"; //TODO set from parameter
    outImage.encoding = sensor_msgs::image_encodings::RGB8;
    outImage.image = output_;
    imagePublisher_.publish(outImage.toImageMsg());
}

void DepthImageProjectorNode::depthToColorCallback(const realsense2_camera::Extrinsics::ConstPtr& depthToColorMsg)
{
    auto& r = depthToColorMsg->rotation;
    auto& t = depthToColorMsg->translation;
    latestDepthToColor_ = {
        static_cast<float>(r[0]), static_cast<float>(r[1]), static_cast<float>(r[2]), 0, 
        static_cast<float>(r[3]), static_cast<float>(r[4]), static_cast<float>(r[5]), 0, 
        static_cast<float>(r[6]), static_cast<float>(r[7]), static_cast<float>(r[8]), 0, 
        static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2]), 1,
    }; //column major order

    depthToColorArrived_ = true;
}

void DepthImageProjectorNode::run()
{
    ros::spin();
}