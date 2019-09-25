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
    const int colorWidth_, colorHeight_;
    const int depthWidth_, depthHeight_;
    const int gridMapWidth_, gridMapHeight_;

    std::array<cgs::gl::Shader, 2> shaders_;
    cgs::gl::Program program_;
    std::vector<Vertex> verticies_;
    cgs::gl::VertexBuffer<Vertex> vbo_;
    cgs::gl::VertexArray vao_;
    cgs::gl::Texture2D colorIn_, depthIn_, textureOut_;
    cgs::gl::Sampler colorSampler_, depthSampler_;
    cgs::gl::FrameBuffer fbo_;

    Impl(int colorWidth, int colorHeight, 
        int depthWidth, int depthHeight,
        int gridMapWidth, int gridMapHeight, float gridMapResolution, 
        float gridMapLayerHeight, float gridMapAccumulationWeight,
        const std::string& vertexShader, const std::string& fragmentShader);

    void project(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth);
};

DepthImageProjector::Impl::Impl(
    int colorWidth, int colorHeight, 
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, 
    float gridMapLayerHeight, float gridMapAccumulationWeight,
    const std::string& vertexShader, const std::string& fragmentShader) : 
    context_(true),
    colorWidth_(colorWidth), colorHeight_(colorHeight), 
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
    colorIn_(GL_SRGB8, colorWidth_, colorHeight_),  
    depthIn_(GL_R16UI, depthWidth_, depthHeight_),  
    textureOut_(GL_RGB8, gridMapWidth, gridMapHeight),
    colorSampler_(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE),
    depthSampler_(GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE),
    fbo_(textureOut_)
{
    //Shaders setup
    program_.use();
    glUniform2f(glGetUniformLocation(program_.get(), "colorSize"), colorWidth, colorHeight);
    glUniform2f(glGetUniformLocation(program_.get(), "depthSize"), depthWidth, depthHeight);
    glUniform1f(glGetUniformLocation(program_.get(), "depthUnit"), 0.001); //TODO change to be set with an external parameter
    glUniform1i(glGetUniformLocation(program_.get(), "colorTexture"), 0);
    glUniform1i(glGetUniformLocation(program_.get(), "depthTexture"), 1);
    glUniform2f(glGetUniformLocation(program_.get(), "gridMapSize"), gridMapWidth_, gridMapHeight_);
    glUniform1f(glGetUniformLocation(program_.get(), "gridMapResolution"), gridMapResolution);
    glUniform1f(glGetUniformLocation(program_.get(), "gridMapLayerHeight"), gridMapLayerHeight);
    glUniform1f(glGetUniformLocation(program_.get(), "gridMapAccumulationWeight"), gridMapAccumulationWeight);

    //Verticies setup
    vao_.mapVariable(vbo_, glGetAttribLocation(program_.get(), "position"), 3, GL_FLOAT, 0);

    //Enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    glBlendEquation(GL_FUNC_ADD);
}

void DepthImageProjector::Impl::project(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth) //TODO add projection matrix argument
{
    if (gridMapWidth_ != dest.cols || gridMapHeight_ != dest.rows || CV_8UC3 != dest.type())
    {
        ROS_ERROR_STREAM(
            "Destination image resolution does not match."                            << std::endl <<
            "width:     texture=" << gridMapWidth_  << ", input=" << dest.cols        << std::endl <<
            "height:    texture=" << gridMapHeight_ << ", input=" << dest.rows        << std::endl <<
            "channel:   texture=" << 3              << ", input=" << dest.channels()  << std::endl <<
            "elemSize1: texture=" << 1              << ", input=" << dest.elemSize1() << std::endl <<
            "type:      texture=" << CV_8UC3        << ", input=" << dest.type());
        return;
    }

    if (depthWidth_ != depth.cols || depthHeight_ != depth.rows || CV_16UC1 != depth.type())
    {
        ROS_ERROR_STREAM(
            "depth image resolution does not match."                                 << std::endl <<
            "width:     texture=" << depthWidth_  << ", input=" << depth.cols        << std::endl <<
            "height:    texture=" << depthHeight_ << ", input=" << depth.rows        << std::endl <<
            "channel:   texture=" << 1            << ", input=" << depth.channels()  << std::endl <<
            "elemSize1: texture=" << 2            << ", input=" << depth.elemSize1() << std::endl <<
            "type:      texture=" << CV_16UC1     << ", input=" << depth.type());
        return;
    }

    if (colorWidth_ != color.cols || colorHeight_ != color.rows || CV_8UC3 != color.type())
    {
        ROS_ERROR_STREAM(
            "color image resolution does not match."                                 << std::endl <<
            "width:     texture=" << colorWidth_  << ", input=" << color.cols        << std::endl <<
            "height:    texture=" << colorHeight_ << ", input=" << color.rows        << std::endl <<
            "channel:   texture=" << 3            << ", input=" << color.channels()  << std::endl <<
            "elemSize1: texture=" << 1            << ", input=" << color.elemSize1() << std::endl <<
            "type:      texture=" << CV_8UC3      << ", input=" << color.type());
        return;
    }

    //TODO update projection matrix
    glUniform2f(glGetUniformLocation(program_.get(), "depthFocalLength"), 323.7779846191406, 323.7779846191406);
    glUniform2f(glGetUniformLocation(program_.get(), "depthCenter"), 322.2593078613281, 182.6495361328125);
    glUniform2f(glGetUniformLocation(program_.get(), "colorFocalLength"), 463.1402587890625, 463.0929870605469);
    glUniform2f(glGetUniformLocation(program_.get(), "colorCenter"), 320.7187194824219, 176.80926513671875);

    //extrinsics matrix in column major order
    //TODO update the matrix using depth_to_color message
    //Note that the rotation variable in the message is column major order
    std::array<float, 16> depthToColorT = {
        0.9999825954437256  , 0.0040609221905469894 , -0.004279938992112875 , 0, 
        -0.00407175999134779, 0.9999884963035583    , -0.0025265226140618324, 0, 
        0.004269629716873169, 0.0025439055170863867 , 0.9999876618385315    , 0, 
        0.014667361974716187, 0.00030949467327445745, 0.0012093170080333948 , 1,
    };
    glUniformMatrix4fv(glGetUniformLocation(program_.get(), "depthToColor"), 1, false, depthToColorT.data());

    //Perform rendering
    colorIn_.write(GL_RGB, GL_UNSIGNED_BYTE, color.data);
    colorIn_.bindToUnit(0);
    colorSampler_.bindToUnit(0);

    depthIn_.write(GL_RED_INTEGER, GL_UNSIGNED_SHORT, depth.data);
    depthIn_.bindToUnit(1);
    depthSampler_.bindToUnit(1);

    fbo_.bind();
    glViewport(0, 0, gridMapWidth_, gridMapHeight_);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);

    vao_.bind();
    glDrawArrays(GL_POINTS, 0, verticies_.size());

    glFinish();

    //Read result
    textureOut_.read(GL_RGB, GL_UNSIGNED_BYTE, dest.data, dest.rows * dest.cols * dest.channels());
}

DepthImageProjector::DepthImageProjector(
    int colorWidth, int colorHeight, 
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, 
    float gridMapLayerHeight, float gridMapAccumulationWeight, 
    const std::string& vertexShader, const std::string& fragmentShader)
try
    : impl_(std::make_unique<DepthImageProjector::Impl>(
        colorWidth, colorHeight, 
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

void DepthImageProjector::project(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth)
{
    impl_->project(dest, color, depth);
}