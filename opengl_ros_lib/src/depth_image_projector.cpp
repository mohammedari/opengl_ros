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

    constexpr Vertex(float x, float y, float z) : 
        x(x), y(y), z(z)
    {}
};

struct DepthImageProjector::Impl
{
    static constexpr std::array<Vertex, 4> SQUARE_VERTICIES = {
        Vertex(-1.f, -1.f, 0.0f),
        Vertex( 1.f, -1.f, 0.0f),
        Vertex( 1.f,  1.f, 0.0f),
        Vertex(-1.f,  1.f, 0.0f),
    };

    static constexpr std::array<uint, 6> SQUARE_INDICIES = {
        0, 1, 2, 
        2, 3, 0,
    };

    OpenGLContext context_;
    const int depthWidth_, depthHeight_;
    const int gridMapWidth_, gridMapHeight_;
    const float gridMapResolution_;
    const float gridMapLayerHeight_;
    const float gridMapAccumulationWeight_;
    const float gridMapDecay_;
    const float minDepth_;
    const float maxDepth_;
    const float depthHitThreshold_;
    const int unknownDepthColor_; 

    //first path
    std::array<cgs::gl::Shader, 3> shaders_;
    cgs::gl::Program program_;
    std::vector<Vertex> verticies_;
    cgs::gl::VertexBuffer<Vertex> vbo_;
    cgs::gl::VertexArray vao_;
    cgs::gl::Texture2D depthIn_;
    cgs::gl::Sampler depthSampler_;
    cgs::gl::Texture2D intermediateBuffer_;
    cgs::gl::Sampler intermediateBufferSampler_;
    cgs::gl::FrameBuffer fbo_;

    //second path
    std::array<cgs::gl::Shader, 2> shaderScaling_;
    cgs::gl::Program programScaling_;
    cgs::gl::VertexBuffer<Vertex> vboScaling_;
    cgs::gl::ElementBuffer<uint> eboScaling_;
    cgs::gl::VertexArray vaoScaling_;
    std::array<cgs::gl::Texture2D, 2> accumulationBuffer_;
    cgs::gl::Sampler accumulationBufferSampler_;
    cgs::gl::Texture2D textureOut_;
    std::array<cgs::gl::FrameBuffer, 2> fboScaling_;
    int currentBuffer_ = 0;
    int previousBuffer_ = 1;

    //not const parameters
    std::array<float, 2> depthFocalLength_ = {};
    std::array<float, 2> depthCenter_ = {}; 
    std::array<float, 16> depthToMap_ = {{0.0f, -1.0f,  0.0f, 0.0f,
                                          0.0f,  0.0f, -1.0f, 0.0f,
                                          1.0f,  0.0f,  0.0f, 0.0f,
                                          0.0f,  0.0f,  0.0f, 1.0f}};
    std::array<float, 16> mapToPreviousMap_ = {{0.0f, -1.0f,  0.0f, 0.0f,
                                                0.0f,  0.0f, -1.0f, 0.0f,
                                                1.0f,  0.0f,  0.0f, 0.0f,
                                                0.0f,  0.0f,  0.0f, 1.0f}};

    Impl(int depthWidth, int depthHeight,
        int gridMapWidth, int gridMapHeight, float gridMapResolution, 
        float gridMapLayerHeight, float gridMapAccumulationWeight, float gridMapDecay_,
        float minDepth, float maxDepth, float depthHitThreshold, int unknownDepthColor, 
        const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader, 
        const std::string& vertexScalingShader, const std::string& fragmentScalingShader);

    void updateProjectionMatrix(
        const std::array<float, 2>& depthFocalLength, const std::array<float, 2>& depthCenter, 
        const std::array<float, 16>& depthToMap, const std::array<float, 16>& mapToPreviousMap);
    void project(cv::Mat& dest, const cv::Mat& depth);
};

constexpr std::array<Vertex, 4> DepthImageProjector::Impl::SQUARE_VERTICIES;
constexpr std::array<uint, 6>   DepthImageProjector::Impl::SQUARE_INDICIES;

DepthImageProjector::Impl::Impl(
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, 
    float gridMapLayerHeight, float gridMapAccumulationWeight, float gridMapDecay,
    float minDepth, float maxDepth, float depthHitThreshold, int unknownDepthColor, 
    const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader, 
    const std::string& vertexScalingShader, const std::string& fragmentScalingShader) :
    context_(true),
    depthWidth_(depthWidth), depthHeight_(depthHeight), 
    gridMapWidth_(gridMapWidth), gridMapHeight_(gridMapHeight), gridMapResolution_(gridMapResolution), 
    gridMapLayerHeight_(gridMapLayerHeight), gridMapAccumulationWeight_(gridMapAccumulationWeight),gridMapDecay_(gridMapDecay),
    minDepth_(minDepth), maxDepth_(maxDepth), depthHitThreshold_(depthHitThreshold), unknownDepthColor_(unknownDepthColor), 
    //first path
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
    depthIn_(GL_R16UI, depthWidth_, depthHeight_),  
    depthSampler_(GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE),
    intermediateBuffer_(GL_RGBA32F, gridMapWidth, gridMapHeight),
    intermediateBufferSampler_(GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE),
    fbo_(intermediateBuffer_),
    //second path
    shaderScaling_({
        cgs::gl::Shader(GL_VERTEX_SHADER,   vertexScalingShader),
        cgs::gl::Shader(GL_FRAGMENT_SHADER, fragmentScalingShader),
    }),
    programScaling_(shaderScaling_),
    vboScaling_(SQUARE_VERTICIES, GL_STATIC_DRAW), 
    eboScaling_(SQUARE_INDICIES, GL_STATIC_DRAW), 
    accumulationBuffer_({
        cgs::gl::Texture2D(GL_RGBA32F, gridMapWidth, gridMapHeight),
        cgs::gl::Texture2D(GL_RGBA32F, gridMapWidth, gridMapHeight),
    }),
    accumulationBufferSampler_(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, {0, 0, 0, 0}),
    textureOut_(GL_R8, gridMapWidth, gridMapHeight),
    fboScaling_({
        cgs::gl::FrameBuffer(textureOut_, accumulationBuffer_[0]),
        cgs::gl::FrameBuffer(textureOut_, accumulationBuffer_[1]),
    })
{
    //Clear accumulation bufffer
    std::vector<float> zeros(gridMapWidth * gridMapHeight * 4, 0);
    for (auto& buffer : accumulationBuffer_)
        buffer.write(GL_RGBA, GL_FLOAT, zeros.data());

    //Shaders setup
    program_.use();
}

void DepthImageProjector::Impl::updateProjectionMatrix(
    const std::array<float, 2>& depthFocalLength, const std::array<float, 2>& depthCenter, 
    const std::array<float, 16>& depthToMap, const std::array<float, 16>& mapToPreviousMap)
{
    depthFocalLength_ = depthFocalLength;
    depthCenter_ = depthCenter;
    depthToMap_ = depthToMap;
    mapToPreviousMap_ = mapToPreviousMap;
}

void DepthImageProjector::Impl::project(cv::Mat& dest, const cv::Mat& depth)
{
    if (gridMapWidth_ != dest.cols || gridMapHeight_ != dest.rows || CV_8UC1 != dest.type())
    {
        ROS_ERROR_STREAM(
            "Destination image resolution does not match."                            << std::endl <<
            "width:     texture=" << gridMapWidth_  << ", input=" << dest.cols        << std::endl <<
            "height:    texture=" << gridMapHeight_ << ", input=" << dest.rows        << std::endl <<
            "channel:   texture=" << 1              << ", input=" << dest.channels()  << std::endl <<
            "elemSize1: texture=" << 1              << ", input=" << dest.elemSize1() << std::endl <<
            "type:      texture=" << CV_8UC1        << ", input=" << dest.type());
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

    //first path
    {
        //Shaders setup
        program_.use();

        glUniform2f(glGetUniformLocation(program_.get() , "depthSize"), depthWidth_, depthHeight_);
        glUniform1f(glGetUniformLocation(program_.get() , "depthUnit"), 0.001); //TODO change to be set with an external parameter
        glUniform1i(glGetUniformLocation(program_.get() , "depthTexture"), 0);
        glUniform2f(glGetUniformLocation(program_.get() , "gridMapSize"), gridMapWidth_, gridMapHeight_);
        glUniform1f(glGetUniformLocation(program_.get() , "gridMapResolution"), gridMapResolution_);
        glUniform1f(glGetUniformLocation(program_.get() , "gridMapLayerHeight"), gridMapLayerHeight_);
        glUniform1f(glGetUniformLocation(program_.get() , "gridMapAccumulationWeight"), gridMapAccumulationWeight_);
        glUniform2f(glGetUniformLocation(program_.get() , "validDepthInMeter"), minDepth_, maxDepth_);
        glUniform1f(glGetUniformLocation(program_.get() , "depthHitThreshold"), depthHitThreshold_);
        glUniform2fv(glGetUniformLocation(program_.get(), "depthFocalLength") , 1, depthFocalLength_.data());
        glUniform2fv(glGetUniformLocation(program_.get(), "depthCenter")      , 1, depthCenter_.data());
        glUniformMatrix4fv(glGetUniformLocation(program_.get(), "depthToMap"), 1, GL_FALSE, depthToMap_.data());

        std::array<float, 4> normalized_camera_pos = {
          depthToMap_[12] / gridMapResolution_ / gridMapWidth_ * 2,
          depthToMap_[13] / gridMapResolution_ / gridMapHeight_ * 2,
          depthToMap_[14] / gridMapLayerHeight_ * 2,
          depthToMap_[15]};
        glUniform4fv(glGetUniformLocation(program_.get(), "lineOrigin"), 1, normalized_camera_pos.data());

        //Verticies setup
        vao_.mapVariable(vbo_, glGetAttribLocation(program_.get(), "input_pixel"), 3, GL_FLOAT, 0);

        //Setup depth value mapping parameter
        glDepthRange(-gridMapLayerHeight_ / 2, gridMapLayerHeight_ / 2);
        glEnable(GL_DEPTH_CLAMP); //disable depth clipping for processing verticies of all height

        //Enable blending
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glBlendEquation(GL_FUNC_ADD);

        depthIn_.write(GL_RED_INTEGER, GL_UNSIGNED_SHORT, depth.data);
        depthIn_.bindToUnit(0);
        depthSampler_.bindToUnit(0);

        fbo_.bind();
        glViewport(0, 0, gridMapWidth_, gridMapHeight_);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT);

        vao_.bind();
        glDrawArrays(GL_POINTS, 0, verticies_.size());
    }

    //second path
    {
        //Shaders setup
        programScaling_.use();
        glUniform2f(glGetUniformLocation(programScaling_.get(), "resolution"), gridMapWidth_, gridMapHeight_);
        glUniform1i(glGetUniformLocation(programScaling_.get(), "texture"), 0);
        glUniform1i(glGetUniformLocation(programScaling_.get(), "accumulationTexture"), 1);
        glUniform1i(glGetUniformLocation(programScaling_.get(), "unknownDepthColor"), unknownDepthColor_);
        glUniform1f(glGetUniformLocation(programScaling_.get(), "gridMapDecay"), gridMapDecay_);
        glUniform1f(glGetUniformLocation(programScaling_.get(), "gridMapResolution"), gridMapResolution_);
        glUniformMatrix4fv(glGetUniformLocation(programScaling_.get(), "mapToPreviousMap"), 1, GL_FALSE, mapToPreviousMap_.data());

        //Verticies setup
        vaoScaling_.mapVariable(vboScaling_, glGetAttribLocation(programScaling_.get(), "position"), 3, GL_FLOAT, 0);
        vaoScaling_.mapVariable(eboScaling_);

        //Disable flags
        glDisable(GL_DEPTH_CLAMP); 
        glDisable(GL_BLEND);

        intermediateBuffer_.bindToUnit(0);
        intermediateBufferSampler_.bindToUnit(0);
        accumulationBuffer_[previousBuffer_].bindToUnit(1);
        accumulationBufferSampler_.bindToUnit(1);

        fboScaling_[currentBuffer_].bind();
        glViewport(0, 0, gridMapWidth_, gridMapHeight_);

        vaoScaling_.bind();
        glDrawElements(GL_TRIANGLES, SQUARE_INDICIES.size(), GL_UNSIGNED_INT, nullptr);
    }

    glFinish();

    //Read result
    textureOut_.read(GL_RED, GL_UNSIGNED_BYTE, dest.data, dest.rows * dest.cols * dest.channels());

    //Swap buffers
    std::swap(currentBuffer_, previousBuffer_);
}

DepthImageProjector::DepthImageProjector(
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, float gridMapResolution, 
    float gridMapLayerHeight, float gridMapAccumulationWeight, float gridMapDecay,
    float minDepth, float maxDepth, float depthHitThreshold, int unknownDepthColor, 
    const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader,
    const std::string& vertexScalingShader, const std::string& fragmentScalingShader)
try
    : impl_(std::make_unique<DepthImageProjector::Impl>(
        depthWidth, depthHeight, 
        gridMapWidth, gridMapHeight, gridMapResolution, 
        gridMapLayerHeight, gridMapAccumulationWeight, gridMapDecay, 
        minDepth, maxDepth, depthHitThreshold, unknownDepthColor, 
        vertexShader, geometryShader, fragmentShader, 
        vertexScalingShader, fragmentScalingShader))
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

void DepthImageProjector::updateProjectionMatrix(
    const std::array<float, 2>& depthFocalLength, const std::array<float, 2>& depthCenter, 
    const std::array<float, 16>& depthToMap, const std::array<float, 16>& mapToPreviousMap)
{
    impl_->updateProjectionMatrix(
        depthFocalLength, depthCenter, 
        depthToMap, mapToPreviousMap);
}

void DepthImageProjector::project(cv::Mat& dest, const cv::Mat& depth)
{
    impl_->project(dest, depth);
}
