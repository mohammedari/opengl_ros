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

    std::array<cgs::gl::Shader, 3> shaders_;
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
        const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader);

    void updateProjectionMatrix(
        const std::array<float, 2> colorFocalLength, const std::array<float, 2> colorCenter, 
        const std::array<float, 2> depthFocalLength, const std::array<float, 2> depthCenter, 
        const std::array<float, 16> depthToColor);
    void project(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth);
};

DepthImageProjector::Impl::Impl(
    int colorWidth, int colorHeight, 
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, 
    float gridMapLayerHeight, float gridMapAccumulationWeight,
    const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader) : 
    context_(true),
    colorWidth_(colorWidth), colorHeight_(colorHeight), 
    depthWidth_(depthWidth), depthHeight_(depthHeight), 
    gridMapWidth_(gridMapWidth), gridMapHeight_(gridMapHeight), 
    shaders_({
        cgs::gl::Shader(GL_VERTEX_SHADER,   vertexShader),
        cgs::gl::Shader(GL_GEOMETRY_SHADER, geometryShader), 
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
    colorSampler_(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER),
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
    vao_.mapVariable(vbo_, glGetAttribLocation(program_.get(), "input_pixel"), 3, GL_FLOAT, 0);

    //Setup depth value mapping parameter
    glDepthRange(-gridMapLayerHeight / 2, gridMapLayerHeight / 2);
    glEnable(GL_DEPTH_CLAMP); //disable depth clipping for processing verticies of all height

    //Enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    glBlendEquation(GL_FUNC_ADD);
}

void DepthImageProjector::Impl::updateProjectionMatrix(
    const std::array<float, 2> colorFocalLength, const std::array<float, 2> colorCenter, 
    const std::array<float, 2> depthFocalLength, const std::array<float, 2> depthCenter, 
    const std::array<float, 16> depthToColor)
{
    glUniform2fv(glGetUniformLocation(program_.get(), "colorFocalLength")  , 1, colorFocalLength.data());
    glUniform2fv(glGetUniformLocation(program_.get(), "colorCenter")       , 1, colorCenter.data());
    glUniform2fv(glGetUniformLocation(program_.get(), "depthFocalLength")  , 1, depthFocalLength.data());
    glUniform2fv(glGetUniformLocation(program_.get(), "depthCenter")       , 1, depthCenter.data());
    glUniformMatrix4fv(glGetUniformLocation(program_.get(), "depthToColor"), 1, false, depthToColor.data());
}

void DepthImageProjector::Impl::project(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth)
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
    const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader)
try
    : impl_(std::make_unique<DepthImageProjector::Impl>(
        colorWidth, colorHeight, 
        depthWidth, depthHeight, 
        gridMapWidth, gridMapHeight, gridMapResolution, 
        gridMapLayerHeight, gridMapAccumulationWeight, 
        vertexShader, geometryShader, fragmentShader))
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

void DepthImageProjector::uniform(const std::string& name, float v1) { 
    glUniform1f(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1);
}
void DepthImageProjector::uniform(const std::string& name, float v1, float v2) { 
    glUniform2f(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2);
}
void DepthImageProjector::uniform(const std::string& name, float v1, float v2, float v3) { 
    glUniform3f(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3);
}
void DepthImageProjector::uniform(const std::string& name, float v1, float v2, float v3, float v4) { 
    glUniform4f(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3, v4);
}
void DepthImageProjector::uniform(const std::string& name, int v1) { 
    glUniform1i(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1);
}
void DepthImageProjector::uniform(const std::string& name, int v1, int v2) { 
    glUniform2i(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2);
}
void DepthImageProjector::uniform(const std::string& name, int v1, int v2, int v3) { 
    glUniform3i(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3);
}
void DepthImageProjector::uniform(const std::string& name, int v1, int v2, int v3, int v4) { 
    glUniform4i(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3, v4);
}
void DepthImageProjector::uniform(const std::string& name, unsigned int v1) { 
    glUniform1ui(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1);
}
void DepthImageProjector::uniform(const std::string& name, unsigned int v1, unsigned int v2) { 
    glUniform2ui(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2);
}
void DepthImageProjector::uniform(const std::string& name, unsigned int v1, unsigned int v2, unsigned int v3) { 
    glUniform3ui(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3);
}
void DepthImageProjector::uniform(const std::string& name, unsigned int v1, unsigned int v2, unsigned int v3, unsigned int v4) { 
    glUniform4ui(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3, v4);
}

void DepthImageProjector::updateProjectionMatrix(
    const std::array<float, 2> colorFocalLength, const std::array<float, 2> colorCenter, 
    const std::array<float, 2> depthFocalLength, const std::array<float, 2> depthCenter, 
    const std::array<float, 16> depthToColor)
{
    impl_->updateProjectionMatrix(
        colorFocalLength, colorCenter, 
        depthFocalLength, depthCenter, 
        depthToColor);
}

void DepthImageProjector::project(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth)
{
    impl_->project(dest, color, depth);
}