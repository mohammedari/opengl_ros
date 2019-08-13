#include "color_extraction_cpu_nodecore.h"

#include <chrono>

#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

ColorExtractionCpuNode::ColorExtractionCpuNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    imagePublisher_  = it_.advertise("image_out", 1);
    imageSubscriber_ = it_.subscribe("image_in" , 1, &ColorExtractionCpuNode::imageCallback, this);

    nh_.param<float>("threshold_l"  , threshold_l_  , 0);
    nh_.param<float>("svm_coef_a"   , svm_coef_a_   , 0);
    nh_.param<float>("svm_coef_b"   , svm_coef_b_   , 0);
    nh_.param<float>("svm_intercept", svm_intercept_, 0);

    int width, height;
    nh_.param<int>("width" , width , 640);
    nh_.param<int>("height", height, 480);

    lab_.create(height, width, CV_8UC3);
    output_.create(height, width, CV_8UC3);
}

void ColorExtractionCpuNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
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
    cv::cvtColor(image, lab_, cv::COLOR_BGR2Lab);
    for (int y = 0; y < lab_.rows; ++y)
        for (int x = 0; x < lab_.cols; ++x)
        {
            const auto lab = lab_.at<cv::Vec3b>(y, x);
            const auto l = lab[0];
            const auto a = lab[1];
            const auto b = lab[2];

            //Apply SVM to extract region
            if (l > threshold_l_ && svm_coef_a_ * a + svm_coef_b_ * b + svm_intercept_ > 0)
            {
                output_.at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(y, x);
                continue;
            }

            //Fill with black if the pixel is not target
            output_.at<cv::Vec3b>(y, x) = { 0, 0, 0 };
        }

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

void ColorExtractionCpuNode::run()
{
    ros::spin();
}