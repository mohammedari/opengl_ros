#ifndef CGS_DEPTH_IMAGE_PROJECTOR_H
#define CGS_DEPTH_IMAGE_PROJECTOR_H

#include <memory>

#include <opencv2/opencv.hpp>

namespace cgs {

/**
 * A class for providing a way to project a depth image to grid maps.
 * This class assumes a depth image from depth cameras like D435, and 
 * project the depth image onto a 2D grid map.
 */
class DepthImageProjector final
{
public:
    DepthImageProjector(
        int depthWidth, int depthHeight, 
        int gridMapWidth, int gridMapHeight, float gridMapResolution, 
        float gridMapLayerHeight, float gridMapAccumulationWeight,
        const std::string& vertexShader, const std::string& fragmentShader);
    ~DepthImageProjector();

    void project(cv::Mat& dest, const cv::Mat& src);

    DepthImageProjector(const DepthImageProjector&) = delete;
    DepthImageProjector& operator=(const DepthImageProjector&) = delete;
    DepthImageProjector(DepthImageProjector&&) = default;
    DepthImageProjector& operator=(DepthImageProjector&&) = default;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} //cgs

#endif