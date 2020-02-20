#ifndef CGS_DEPTH_IMAGE_PROJECTOR_H
#define CGS_DEPTH_IMAGE_PROJECTOR_H

#include <array>
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
        float minDepth, float maxDepth, float depthHitThreshold, int unknownDepthColor, 
        const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader,
        const std::string& vertexScalingShader, const std::string& fragmentScalingShader); 
    ~DepthImageProjector();

    void updateProjectionMatrix(
        const std::array<float, 2> depthFocalLength, const std::array<float, 2> depthCenter, 
        const std::array<float, 16> depthToMap);
    void project(cv::Mat& dest, const cv::Mat& depth);

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
