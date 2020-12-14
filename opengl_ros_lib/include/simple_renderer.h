#ifndef CGS_SIMPLE_RENDERER_H
#define CGS_SIMPLE_RENDERER_H

#include <memory>

#include <opencv2/opencv.hpp>

namespace cgs {

/**
 * A simple renderer class that providse a way to render on an OpenCV texture
 * using arbitary vertex and fragment shader. Any uniform parameters for shaders 
 * can be set with methods. The rendering method takes one OpenCV texture for 
 * referring in the fragment shader, and another texture for writing the result
 * on it.
 */
class SimpleRenderer final
{
public:
    SimpleRenderer(
        int width, int height, 
        const std::string& vertexShader, 
        const std::string& fragmentShader);
    ~SimpleRenderer();

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

    void render(cv::Mat& dest, const cv::Mat& src);
    void render(cv::Mat& dest, const cv::Mat& src, const cv::Mat& secondSrc);

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
