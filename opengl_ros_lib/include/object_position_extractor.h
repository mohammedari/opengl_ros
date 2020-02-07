#ifndef CGS_OBJECT_POSITION_EXTRACTOR_H
#define CGS_OBJECT_POSITION_EXTRACTOR_H

#include <memory>

#include <opencv2/opencv.hpp>

namespace cgs {

/**
 * A class that extract positions of detected object combining color and depth images.
 * After classifying pixels based on color, 3D position of each pixel is stored in the 
 * output image.
 */
class ObjectPositionExtractor final
{
public:
    ObjectPositionExtractor(
        int colorWidth, int colorHeight, 
        int depthWidth, int depthHeight, 
        int outputWidth, int outputHeight,
        float minDepth, float maxDepth,
        const std::string& vertexShader, 
        const std::string& fragmentShader);
    ~ObjectPositionExtractor();

    void updateProjectionMatrix(
        const std::array<float, 2> colorFocalLength, const std::array<float, 2> colorCenter, 
        const std::array<float, 2> depthFocalLength, const std::array<float, 2> depthCenter, 
        const std::array<float, 16> depthToColor);
    void extract(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth);

    ObjectPositionExtractor(const ObjectPositionExtractor&) = delete;
    ObjectPositionExtractor& operator=(const ObjectPositionExtractor&) = delete;
    ObjectPositionExtractor(ObjectPositionExtractor&&) = default;
    ObjectPositionExtractor& operator=(ObjectPositionExtractor&&) = default;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} //cgs

#endif