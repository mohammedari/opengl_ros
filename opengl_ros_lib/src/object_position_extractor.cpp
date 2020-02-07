#include "object_position_extractor.h"

#include <array>

#include <ros/ros.h>

#include "opengl_context.h"
#include "gl/gl.h"

using namespace cgs;

struct Vertex
{
    float x, y, z;

    constexpr Vertex(float x, float y, float z) : 
        x(x), y(y), z(z)
    {}
};

struct ObjectPositionExtractor::Impl
{
    OpenGLContext context_;
    const int colorWidth_, colorHeight_;
    const int depthWidth_, depthHeight_;
    const int outputWidth_, outputHeight_;
    const float minDepth_, maxDepth_;

    std::array<cgs::gl::Shader, 2> shaders_;
    cgs::gl::Program program_;
    std::vector<Vertex> verticies_;
    cgs::gl::VertexBuffer<Vertex> vbo_;
    cgs::gl::VertexArray vao_;
    cgs::gl::Texture2D colorIn_, depthIn_, textureOut_;
    cgs::gl::Sampler colorSampler_, depthSampler_;
    cgs::gl::FrameBuffer fbo_;

    //not const parameters
    std::array<float, 2> colorFocalLength_ = {};
    std::array<float, 2> colorCenter_      = {}; 
    std::array<float, 2> depthFocalLength_ = {};
    std::array<float, 2> depthCenter_      = {}; 
    std::array<float, 16> depthToColor_    = {{0.0f, -1.0f,  0.0f, 0.0f,
                                               0.0f,  0.0f, -1.0f, 0.0f,
                                               1.0f,  0.0f,  0.0f, 0.0f,
                                               0.0f,  0.0f,  0.0f, 1.0f}};

    Impl(
        int colorWidth, int colorHeight, 
        int depthWidth, int depthHeight, 
        int outputWidth, int outputHeight,
        float minDepth, float maxDepth,
        const std::string& vertexShader, 
        const std::string& fragmentShader);

    void updateProjectionMatrix(
        const std::array<float, 2> colorFocalLength, const std::array<float, 2> colorCenter, 
        const std::array<float, 2> depthFocalLength, const std::array<float, 2> depthCenter, 
        const std::array<float, 16> depthToColor);
    void extract(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth);
};

ObjectPositionExtractor::Impl::Impl(
    int colorWidth, int colorHeight, 
    int depthWidth, int depthHeight, 
    int outputWidth, int outputHeight,
    float minDepth, float maxDepth,
    const std::string& vertexShader, 
    const std::string& fragmentShader) : 
    context_(true),
    colorWidth_(colorWidth), colorHeight_(colorHeight), 
    depthWidth_(depthWidth), depthHeight_(depthHeight), 
    outputWidth_(outputWidth), outputHeight_(outputHeight), 
    minDepth_(minDepth), maxDepth_(maxDepth),
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
    textureOut_(GL_RGBA32F, outputWidth_, outputHeight_), 
    colorSampler_(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER),
    depthSampler_(GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE),
    fbo_(textureOut_)
{
    //Shaders setup
    program_.use();
    glUniform2f(glGetUniformLocation(program_.get() , "colorSize"), colorWidth_, colorHeight_);
    glUniform2f(glGetUniformLocation(program_.get() , "depthSize"), depthWidth_, depthHeight_);
    glUniform2f(glGetUniformLocation(program_.get() , "outputSize"), outputWidth_, outputHeight_);
    glUniform1f(glGetUniformLocation(program_.get() , "depthUnit"), 0.001); //TODO change to be set with an external parameter
    glUniform1i(glGetUniformLocation(program_.get() , "colorTexture"), 0);
    glUniform1i(glGetUniformLocation(program_.get() , "depthTexture"), 1);
    glUniform2f(glGetUniformLocation(program_.get() , "validDepthInMeter"), minDepth_, maxDepth_);

    //Verticies setup
    vao_.mapVariable(vbo_, glGetAttribLocation(program_.get(), "inputPixel"), 3, GL_FLOAT, 0);
}

void ObjectPositionExtractor::Impl::updateProjectionMatrix(
    const std::array<float, 2> colorFocalLength, const std::array<float, 2> colorCenter, 
    const std::array<float, 2> depthFocalLength, const std::array<float, 2> depthCenter, 
    const std::array<float, 16> depthToColor)
{
    colorFocalLength_ = colorFocalLength;
    depthFocalLength_ = depthFocalLength;
    colorCenter_ = colorCenter;
    depthCenter_ = depthCenter;
    depthToColor_ = depthToColor;

}

static bool validateImage(const cv::Mat& image, int width, int height, int type)
{
    if (width != image.cols || height != image.rows || type != image.type())
    {
        ROS_ERROR_STREAM(
            "Image resolution does not match."                         << std::endl << 
            "width:  texture=" << width  << ", input=" << image.cols   << std::endl << 
            "height: texture=" << height << ", input=" << image.rows   << std::endl << 
            "type:   texture=" << type   << ", input=" << image.type() << "(" << image.channels() << ", " << image.elemSize1() << ")");
        return false;
    }

    return true;
}

void ObjectPositionExtractor::Impl::extract(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth)
{
    if (!validateImage(dest, outputWidth_, outputHeight_, CV_32FC4)) return;
    if (!validateImage(color, colorWidth_, colorHeight_, CV_8UC3)) return;
    if (!validateImage(depth, depthWidth_, depthHeight_, CV_16UC1)) return;

    //Update parameters
    glUniform2fv(glGetUniformLocation(program_.get(), "colorFocalLength")  , 1, colorFocalLength_.data());
    glUniform2fv(glGetUniformLocation(program_.get(), "colorCenter")       , 1, colorCenter_.data());
    glUniform2fv(glGetUniformLocation(program_.get(), "depthFocalLength")  , 1, depthFocalLength_.data());
    glUniform2fv(glGetUniformLocation(program_.get(), "depthCenter")       , 1, depthCenter_.data());
    glUniformMatrix4fv(glGetUniformLocation(program_.get(), "depthToColor"), 1, false, depthToColor_.data());

    //Perform rendering
    colorIn_.write(GL_RGB, GL_UNSIGNED_BYTE, color.data);
    colorIn_.bindToUnit(0);
    colorSampler_.bindToUnit(0);

    depthIn_.write(GL_RED_INTEGER, GL_UNSIGNED_SHORT, depth.data);
    depthIn_.bindToUnit(1);
    depthSampler_.bindToUnit(1);

    fbo_.bind();
    glViewport(0, 0, outputWidth_, outputHeight_);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);

    vao_.bind();
    glDrawArrays(GL_POINTS, 0, verticies_.size());

    glFinish();

    //Read result
    textureOut_.read(GL_RGBA, GL_FLOAT, dest.data, dest.rows * dest.cols * dest.channels());
}

ObjectPositionExtractor::ObjectPositionExtractor(
    int colorWidth, int colorHeight, 
    int depthWidth, int depthHeight, 
    int outputWidth, int outputHeight,
    float minDepth, float maxDepth,
    const std::string& vertexShader, 
    const std::string& fragmentShader)
try
    : impl_(std::make_unique<ObjectPositionExtractor::Impl>(
        colorWidth, colorHeight, 
        depthWidth, depthHeight, 
        outputWidth, outputHeight,
        minDepth, maxDepth,
        vertexShader, 
        fragmentShader))
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

ObjectPositionExtractor::~ObjectPositionExtractor() = default;

void ObjectPositionExtractor::updateProjectionMatrix(
    const std::array<float, 2> colorFocalLength, const std::array<float, 2> colorCenter, 
    const std::array<float, 2> depthFocalLength, const std::array<float, 2> depthCenter, 
    const std::array<float, 16> depthToColor)
{
    impl_->updateProjectionMatrix(
        colorFocalLength, colorCenter, 
        depthFocalLength, depthCenter, 
        depthToColor);
}

void ObjectPositionExtractor::extract(cv::Mat& dest, const cv::Mat& color, const cv::Mat& depth)
{
    impl_->extract(dest, color, depth);
}