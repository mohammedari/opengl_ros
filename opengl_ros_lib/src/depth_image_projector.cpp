#include "depth_image_projector.h"

#include <array>
#include <vector>

#include <ros/ros.h>

#include "opengl_context.h"
#include "gl/gl.h"

using namespace cgs;

struct Vertex
{
    float x, y, z;
};

struct DepthImageProjector::Impl
{
    static constexpr std::array<Vertex, 4> VERTICIES = {
        -1.f, -1.f, 0.0f,
         1.f, -1.f, 0.0f,
         1.f,  1.f, 0.0f,
        -1.f,  1.f, 0.0f,
    };

    static constexpr std::array<uint, 6> INDICIES = {
        0, 1, 2, 
        2, 3, 0,
    };

    OpenGLContext context_;
    const int depthWidth_, depthHeight_;
    const int gridMapWidth_, gridMapHeight_;

    std::array<cgs::gl::Shader, 2> shaders_;
    cgs::gl::Program program_;
    cgs::gl::VertexBuffer<Vertex> vbo_;
    cgs::gl::ElementBuffer<uint> ebo_;
    cgs::gl::VertexArray vao_;
    cgs::gl::Texture2D textureIn_, textureOut_;
    cgs::gl::Sampler sampler_;
    cgs::gl::FrameBuffer fbo_;

    Impl(int depthWidth, int depthHeight, 
        int gridMapWidth, int gridMapHeight, float gridMapResolution, float gridMapLayerHeight,
        const std::string& vertexShader, const std::string& fragmentShader);

    void project(cv::Mat& dest, const cv::Mat& src);
};

constexpr std::array<Vertex, 4> DepthImageProjector::Impl::VERTICIES;
constexpr std::array<uint, 6> DepthImageProjector::Impl::INDICIES;

DepthImageProjector::Impl::Impl(
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, float gridMapLayerHeight, 
    const std::string& vertexShader, const std::string& fragmentShader) : 
    context_(true),
    depthWidth_(depthWidth), depthHeight_(depthHeight), 
    gridMapWidth_(gridMapWidth), gridMapHeight_(gridMapHeight), 
    shaders_({
        cgs::gl::Shader(GL_VERTEX_SHADER,   vertexShader),
        cgs::gl::Shader(GL_FRAGMENT_SHADER, fragmentShader),
    }), 
    program_(shaders_), 
    vbo_(VERTICIES, GL_STATIC_DRAW), 
    ebo_(INDICIES, GL_STATIC_DRAW), 
    textureIn_(GL_R16, depthWidth_, depthHeight_),  
    textureOut_(GL_R8, gridMapWidth, gridMapHeight),
    sampler_(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE),
    fbo_(textureOut_)
{
    //Shaders setup
    program_.use();
    glUniform2f(glGetUniformLocation(program_.get(), "resolution"), gridMapWidth_, gridMapHeight_);
    glUniform1i(glGetUniformLocation(program_.get(), "texture"), 0);

    //Verticies setup
    vao_.mapVariable(vbo_, glGetAttribLocation(program_.get(), "position"), 3, GL_FLOAT, 0);
    vao_.mapVariable(ebo_);
}

void DepthImageProjector::Impl::project(cv::Mat& dest, const cv::Mat& src)
{
    if (gridMapWidth_ != dest.cols || gridMapHeight_ != dest.rows || CV_8SC1 != dest.type())
    {
        ROS_ERROR_STREAM(
            "Destination image resolution does not match."                            << std::endl <<
            "width:     texture=" << gridMapWidth_  << ", input=" << dest.cols        << std::endl <<
            "height:    texture=" << gridMapHeight_ << ", input=" << dest.rows        << std::endl <<
            "channel:   texture=" << 1              << ", input=" << dest.channels()  << std::endl <<
            "elemSize1: texture=" << 1              << ", input=" << dest.elemSize1() << std::endl <<
            "type:      texture=" << CV_8SC1        << ", input=" << dest.type());
        return;
    }

    if (depthWidth_ != src.cols || depthHeight_ != src.rows || CV_16UC1 != src.type())
    {
        ROS_ERROR_STREAM(
            "Source image resolution does not match."                              << std::endl <<
            "width:     texture=" << depthWidth_  << ", input=" << src.cols        << std::endl <<
            "height:    texture=" << depthHeight_ << ", input=" << src.rows        << std::endl <<
            "channel:   texture=" << 1            << ", input=" << src.channels()  << std::endl <<
            "elemSize1: texture=" << 2            << ", input=" << src.elemSize1() << std::endl <<
            "type:      texture=" << CV_16UC1     << ", input=" << src.type());
        return;
    }

    //Perform rendering
    textureIn_.write(GL_RED, GL_UNSIGNED_SHORT, src.data);
    textureIn_.bindToUnit(0);
    sampler_.bindToUnit(0);

    fbo_.bind();
    glViewport(0, 0, gridMapWidth_, gridMapHeight_);

    vao_.bind();
    glDrawElements(GL_TRIANGLES, INDICIES.size(), GL_UNSIGNED_INT, nullptr);

    glFinish();

    //Read result
    textureOut_.read(GL_RED, GL_BYTE, dest.data, dest.rows * dest.cols * dest.channels());

}

DepthImageProjector::DepthImageProjector(
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, float gridMapLayerHeight, 
    const std::string& vertexShader, const std::string& fragmentShader)
try
    : impl_(std::make_unique<DepthImageProjector::Impl>(
        depthWidth, depthHeight, 
        gridMapWidth, gridMapHeight, gridMapResolution, gridMapLayerHeight, 
        vertexShader, fragmentShader))
{
}
catch (cgs::egl::Exception& e)
{
    ROS_FATAL_STREAM(e.what());
    throw;
}
catch (cgs::gl::Exception& e)
{
    ROS_FATAL_STREAM(e.what());
    throw;
}

DepthImageProjector::~DepthImageProjector() = default;

void DepthImageProjector::project(cv::Mat& dest, const cv::Mat& src)
{
    impl_->project(dest, src);
}