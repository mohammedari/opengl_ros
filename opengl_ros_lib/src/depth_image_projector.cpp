#include "depth_image_projector.h"

#include <array>
#include <vector>

#include <ros/ros.h>

#include "opengl_context.h"
#include "renderdoc_wrapper.h"
#include "gl/gl.h"

using namespace cgs;

struct Vertex
{
    float x, y, z;

    Vertex(float x, float y, float z) : 
        x(x), y(y), z(z)
    {}
};

struct DepthImageProjector::Impl
{
    RenderDoc renderdoc_;
    OpenGLContext context_;
    const int depthWidth_, depthHeight_;
    const int gridMapWidth_, gridMapHeight_;

    std::array<cgs::gl::Shader, 2> shaders_;
    cgs::gl::Program program_;
    std::vector<Vertex> verticies_;
    cgs::gl::VertexBuffer<Vertex> vbo_;
    cgs::gl::VertexArray vao_;
    cgs::gl::Texture2D textureIn_, textureOut_;
    cgs::gl::Sampler sampler_;
    cgs::gl::FrameBuffer fbo_;

    Impl(int depthWidth, int depthHeight, 
        int gridMapWidth, int gridMapHeight, float gridMapResolution, float gridMapLayerHeight,
        const std::string& vertexShader, const std::string& fragmentShader);
    ~Impl();

    void project(cv::Mat& dest, const cv::Mat& src);
};

DepthImageProjector::Impl::Impl(
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, float gridMapLayerHeight, 
    const std::string& vertexShader, const std::string& fragmentShader) : 
    renderdoc_("/home/mohammedari/depth_image_projector"),
    context_(true),
    depthWidth_(depthWidth), depthHeight_(depthHeight), 
    gridMapWidth_(gridMapWidth), gridMapHeight_(gridMapHeight), 
    shaders_({
        cgs::gl::Shader(GL_VERTEX_SHADER,   vertexShader),
        cgs::gl::Shader(GL_FRAGMENT_SHADER, fragmentShader),
    }), 
    program_(shaders_), 
    verticies_([](int width, int height){
        std::vector<Vertex> v;
        for (int i = 0; i < height; ++i)
            for (int j = 0; j < width; ++j)
                v.emplace_back(i, j, 0);
        return v;
    }(gridMapWidth, gridMapHeight)),
    vbo_(verticies_, GL_STATIC_DRAW), 
    textureIn_(GL_R16, depthWidth_, depthHeight_),  
    textureOut_(GL_R8, gridMapWidth, gridMapHeight),
    sampler_(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE),
    fbo_(textureOut_)
{
    //Shaders setup
    program_.use();
    glUniform2f(glGetUniformLocation(program_.get(), "depthSize"), depthWidth, depthHeight);
    glUniform1i(glGetUniformLocation(program_.get(), "texture"), 0);
    glUniform2f(glGetUniformLocation(program_.get(), "gridMapSize"), gridMapWidth_, gridMapHeight_);
    glUniform1f(glGetUniformLocation(program_.get(), "gridMapResolution"), gridMapResolution);
    glUniform1f(glGetUniformLocation(program_.get(), "gridMapLayerHeight"), gridMapLayerHeight);

    //Verticies setup
    vao_.mapVariable(vbo_, glGetAttribLocation(program_.get(), "position"), 3, GL_FLOAT, 0);

    renderdoc_.StartFrameCapture(context_);
}

DepthImageProjector::Impl::~Impl()
{
    renderdoc_.EndFrameCapture(context_);
}

void DepthImageProjector::Impl::project(cv::Mat& dest, const cv::Mat& src) //TODO add projection matrix argument
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

    //TODO update projection matrix
    glUniform2f(glGetUniformLocation(program_.get(), "focalLength"), 0, 0);
    glUniform2f(glGetUniformLocation(program_.get(), "center"), 0, 0);

    //Perform rendering
    textureIn_.write(GL_RED, GL_UNSIGNED_SHORT, src.data);
    textureIn_.bindToUnit(0);
    sampler_.bindToUnit(0);

    fbo_.bind();
    glViewport(0, 0, gridMapWidth_, gridMapHeight_);

    vao_.bind();
    glDrawArrays(GL_POINTS, 0, verticies_.size());

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