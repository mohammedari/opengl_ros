#ifndef CGS_SIMPLE_RENDERER_H
#define CGS_SIMPLE_RENDERER_H

#include <memory>

#include <opencv2/opencv.hpp>

namespace cgs {

class SimpleRenderer final
{
public:
    SimpleRenderer(
        int width, int height, 
        const std::string& vertexShader, 
        const std::string& fragmentShader);
    ~SimpleRenderer();

    void render(cv::Mat& dest, const cv::Mat& src);

    SimpleRenderer(const SimpleRenderer&) = delete;
    SimpleRenderer& operator=(const SimpleRenderer&) = delete;
    SimpleRenderer(SimpleRenderer&&) = default;
    SimpleRenderer& operator=(SimpleRenderer&&) = default;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} //cgs

#endif