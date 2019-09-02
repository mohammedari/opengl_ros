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

    Vertex(float x, float y, float z) : 
        x(x), y(y), z(z)
    {}
};

struct DepthImageProjector::Impl
{
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
        int gridMapWidth, int gridMapHeight, float gridMapResolution, 
        float gridMapLayerHeight, float gridMapAccumulationWeight,
        const std::string& vertexShader, const std::string& fragmentShader);

    void project(cv::Mat& dest, const cv::Mat& src);
};

DepthImageProjector::Impl::Impl(
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, 
    float gridMapLayerHeight, float gridMapAccumulationWeight,
    const std::string& vertexShader, const std::string& fragmentShader) : 
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
        for (int j = 0; j < height; ++j)
            for (int i = 0; i < width; ++i)
                v.emplace_back(i, j, 0);
        return v;
    }(depthWidth, depthHeight)),
    vbo_(verticies_, GL_STATIC_DRAW), 
    textureIn_(GL_R16UI, depthWidth_, depthHeight_),  
    textureOut_(GL_R8I, gridMapWidth, gridMapHeight),
    sampler_(GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE),
    fbo_(textureOut_)
{
    //Shaders setup
    program_.use();
    glUniform2f(glGetUniformLocation(program_.get(), "depthSize"), depthWidth, depthHeight);
    glUniform1f(glGetUniformLocation(program_.get(), "depthUnit"), 0.001); //TODO change to be set external parameter
    glUniform1i(glGetUniformLocation(program_.get(), "depthTexture"), 0);
    glUniform2f(glGetUniformLocation(program_.get(), "gridMapSize"), gridMapWidth_, gridMapHeight_);
    glUniform1f(glGetUniformLocation(program_.get(), "gridMapResolution"), gridMapResolution);
    glUniform1f(glGetUniformLocation(program_.get(), "gridMapLayerHeight"), gridMapLayerHeight);
    glUniform1f(glGetUniformLocation(program_.get(), "gridMapAccumulationWeight"), gridMapAccumulationWeight);

    //Verticies setup
    vao_.mapVariable(vbo_, glGetAttribLocation(program_.get(), "position"), 3, GL_FLOAT, 0);
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
    glUniform2f(glGetUniformLocation(program_.get(), "focalLength"), 323.7779846191406, 323.7779846191406);
    glUniform2f(glGetUniformLocation(program_.get(), "center"), 322.2593078613281, 182.6495361328125);

    //Perform rendering
    textureIn_.write(GL_RED_INTEGER, GL_UNSIGNED_SHORT, src.data);
    textureIn_.bindToUnit(0);
    sampler_.bindToUnit(0);

    fbo_.bind();
	glClearColor(0 , 0 , 0 , 0);
	glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, gridMapWidth_, gridMapHeight_);

    vao_.bind();
    glDrawArrays(GL_POINTS, 0, verticies_.size());

    glFinish();

    //Read result
    textureOut_.read(GL_RED_INTEGER, GL_BYTE, dest.data, dest.rows * dest.cols * dest.channels());
}

DepthImageProjector::DepthImageProjector(
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, 
    float gridMapLayerHeight, float gridMapAccumulationWeight, 
    const std::string& vertexShader, const std::string& fragmentShader)
try
    : impl_(std::make_unique<DepthImageProjector::Impl>(
        depthWidth, depthHeight, 
        gridMapWidth, gridMapHeight, gridMapResolution, 
        gridMapLayerHeight, gridMapAccumulationWeight, 
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