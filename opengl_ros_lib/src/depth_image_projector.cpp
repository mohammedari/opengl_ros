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
        int gridMapWidth, int gridMapHeight, int gridMapLayerHeight,
        const std::string& vertexShader, 
        const std::string& fragmentShader);

    void project(cv::Mat& dest, const cv::Mat& depth);
};

constexpr std::array<Vertex, 4> DepthImageProjector::Impl::VERTICIES;
constexpr std::array<uint, 6> DepthImageProjector::Impl::INDICIES;

DepthImageProjector::Impl::Impl(
    int depthWidth, int depthHeight, 
    int gridMapWidth, int gridMapHeight, int gridMapLayerHeight, 
    const std::string& vertexShader, 
    const std::string& fragmentShader) : 
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
    textureIn_(GL_R16UI, depthWidth_, depthHeight_),  
    textureOut_(GL_R8UI, gridMapWidth, gridMapHeight), //TODO specify appropriate parameter
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