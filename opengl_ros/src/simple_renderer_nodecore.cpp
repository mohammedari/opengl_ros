#include "simple_renderer_nodecore.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

void SimpleRendererNode::reconfigure_callback(opengl_ros::opengl_rosConfig &config, uint32_t level) {
    threshold_l_   = config.threshold_l; 
    svm_coef_a_    = config.svm_coef_a;  
    svm_coef_b_    = config.svm_coef_b;  
    svm_intercept_ = config.svm_intercept;

    renderer_->uniform("threshold_l"  , threshold_l_);
    renderer_->uniform("svm_coef_a"   , svm_coef_a_);
    renderer_->uniform("svm_coef_b"   , svm_coef_b_);
    renderer_->uniform("svm_intercept", svm_intercept_);
}

SimpleRendererNode::SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    std::string image_in, image_out;
    nh_.param<std::string>("image_in"  , image_in  , "");
    nh_.param<std::string>("image_out" , image_out  , "");

    imagePublisher_  = it_.advertise("image_out", 1);
    imageSubscriber_ = it_.subscribe("image_in" , 1, &SimpleRendererNode::imageCallback, this);

    int width, height;
    nh_.param<int>("width" , width , 640);
    nh_.param<int>("height", height, 480);

    std::string vertexShader, fragmentShader;
    nh_.param<std::string>("vertex_shader"  , vertexShader  , "");
    nh_.param<std::string>("fragment_shader", fragmentShader, "");

    renderer_ = std::make_unique<cgs::SimpleRenderer>(
        width, height, 
        vertexShader, fragmentShader
    );

    output_.create(height, width, CV_8UC3);
}

void SimpleRendererNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    auto start = std::chrono::system_clock::now();

    const auto& image = cv_ptr->image;
    renderer_->render(output_, image);

    ROS_DEBUG_STREAM(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() << "ns";
    );

    //Publish
    cv_bridge::CvImage outImage;;
    outImage.header = cv_ptr->header;
    outImage.encoding = cv_ptr->encoding;
    outImage.image = output_;
    imagePublisher_.publish(outImage.toImageMsg());
}

void SimpleRendererNode::run()
{
    dynamic_reconfigure::Server<opengl_ros::opengl_rosConfig> server(ros::NodeHandle("color_extraction_params"));
    dynamic_reconfigure::Server<opengl_ros::opengl_rosConfig>::CallbackType f;
    f = boost::bind(boost::mem_fn(&SimpleRendererNode::reconfigure_callback), this, _1, _2);
    server.setCallback(f);

    ros::spin();
}