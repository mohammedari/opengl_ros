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
        int colorWidth, int colorHeight, 
        int depthWidth, int depthHeight, 
        int gridMapWidth, int gridMapHeight, float gridMapResolution, 
        float gridMapLayerHeight, float gridMapAccumulationWeight,
        const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader);
    ~DepthImageProjector();

    void uniform(const std::string& name, float v1);
    void uniform(const std::string& name, float v1, float v2);
    void uniform(const std::string& name, float v1, float v2, float v3);
    void uniform(const std::string& name, float v1, float v2, float v3, float v4);
    void uniform(const std::string& name, int v1);
    void uniform(const std::string& name, int v1, int v2);
    void uniform(const std::string& name, int v1, int v2, int v3);
    void uniform(const std::string& name, int v1, int v2, int v3, int v4);
    void uniform(const std::string& name, unsigned int v1);
    void uniform(const std::string& name, unsigned int v1, unsigned int v2);
    void uniform(const std::string& name, unsigned int v1, unsigned int v2, unsigned int v3);
    void uniform(const std::string& name, unsigned int v1, unsigned int v2, unsigned int v3, unsigned int v4);

    void updateProjectionMatrix(
        const std::array<float, 2> colorFocalLength, const std::array<float, 2> colorCenter, 
        const std::array<float, 2> depthFocalLength, const std::array<float, 2> depthCenter, 
        const std::array<float, 16> depthToColor);
    void project(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth);

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