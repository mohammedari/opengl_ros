#include "simple_renderer.h"

#include <array>

#include <ros/ros.h>

#include "opengl_context.h"
#include "gl/gl.h"

using namespace cgs;

struct Vertex
{
    float x, y, z;
};

struct SimpleRenderer::Impl
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
    const int width_, height_;

    std::array<cgs::gl::Shader, 2> shaders_;
    cgs::gl::Program program_;
    cgs::gl::VertexBuffer<Vertex> vbo_;
    cgs::gl::ElementBuffer<uint> ebo_;
    cgs::gl::VertexArray vao_;
    cgs::gl::Texture2D textureIn_, textureOut_;
    cgs::gl::Sampler sampler_;
    cgs::gl::FrameBuffer fbo_;

    Impl(int width, int height, 
        const std::string& vertexShader, 
        const std::string& fragmentShader);

    void render(cv::Mat& dest, const cv::Mat& src);
};

constexpr std::array<Vertex, 4> SimpleRenderer::Impl::VERTICIES;
constexpr std::array<uint, 6> SimpleRenderer::Impl::INDICIES;

SimpleRenderer::Impl::Impl(
    int width, int height, 
    const std::string& vertexShader, 
    const std::string& fragmentShader) : 
    context_(true),
    width_(width), height_(height), 
    shaders_({
        cgs::gl::Shader(GL_VERTEX_SHADER,   vertexShader),
        cgs::gl::Shader(GL_FRAGMENT_SHADER, fragmentShader),
    }), 
    program_(shaders_), 
    vbo_(VERTICIES, GL_STATIC_DRAW), 
    ebo_(INDICIES, GL_STATIC_DRAW), 
    textureIn_(GL_SRGB8, width_, height_),  //
    textureOut_(GL_SRGB8, width_, height_), //Assuming sRGB input and output
    sampler_(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE),
    fbo_(textureOut_)
{
    //Shaders setup
    program_.use();
    glUniform2f(glGetUniformLocation(program_.get(), "resolution"), width_, height_);
    glUniform1i(glGetUniformLocation(program_.get(), "texture"), 0);

    //Verticies setup
    vao_.mapVariable(vbo_, glGetAttribLocation(program_.get(), "position"), 3, GL_FLOAT, 0);
    vao_.mapVariable(ebo_);
}

void SimpleRenderer::Impl::render(cv::Mat& dest, const cv::Mat& src)
{
    if (width_ != dest.cols || height_ != dest.rows || CV_8UC3 != dest.type())
    {
        ROS_ERROR_STREAM(
            "Destination image resolution does not match."                      << std::endl << 
            "width:     texture=" << width_   << ", input=" << dest.cols        << std::endl << 
            "height:    texture=" << height_  << ", input=" << dest.rows        << std::endl << 
            "channel:   texture=" << 3        << ", input=" << dest.channels()  << std::endl << 
            "elemSize1: texture=" << 1        << ", input=" << dest.elemSize1() << std::endl << 
            "type:      texture=" << CV_8UC3  << ", input=" << dest.type());
        return;
    }

    if (width_ != src.cols || height_ != src.rows || CV_8UC3 != src.type())
    {
        ROS_ERROR_STREAM(
            "Source image resolution does not match."                           << std::endl << 
            "width:     texture=" << width_   << ", input=" << src.cols         << std::endl << 
            "height:    texture=" << height_  << ", input=" << src.rows         << std::endl << 
            "channel:   texture=" << 3        << ", input=" << src.channels()   << std::endl << 
            "elemSize1: texture=" << 1        << ", input=" << src.elemSize1()  << std::endl << 
            "type:      texture=" << CV_8UC3  << ", input=" << src.type());
        return;
    }

    //Perform rendering
    textureIn_.write(GL_BGR, GL_UNSIGNED_BYTE, src.data);
    textureIn_.bindToUnit(0);
    sampler_.bindToUnit(0);

    fbo_.bind();
    glViewport(0, 0, width_, height_);

    vao_.bind();
    glDrawElements(GL_TRIANGLES, INDICIES.size(), GL_UNSIGNED_INT, nullptr);

    glFinish();

    //Read result
    textureOut_.read(GL_BGR, GL_UNSIGNED_BYTE, dest.data, dest.rows * dest.cols * dest.channels());
}

SimpleRenderer::SimpleRenderer(
    int width, int height, 
    const std::string& vertexShader, 
    const std::string& fragmentShader)
try
    : impl_(std::make_unique<SimpleRenderer::Impl>(width, height, vertexShader, fragmentShader))
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

SimpleRenderer::~SimpleRenderer() = default;

void SimpleRenderer::uniform(const std::string& name, float v1) { 
    glUniform1f(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1);
}
void SimpleRenderer::uniform(const std::string& name, float v1, float v2) { 
    glUniform2f(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2);
}
void SimpleRenderer::uniform(const std::string& name, float v1, float v2, float v3) { 
    glUniform3f(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3);
}
void SimpleRenderer::uniform(const std::string& name, float v1, float v2, float v3, float v4) { 
    glUniform4f(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3, v4);
}
void SimpleRenderer::uniform(const std::string& name, int v1) { 
    glUniform1i(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1);
}
void SimpleRenderer::uniform(const std::string& name, int v1, int v2) { 
    glUniform2i(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2);
}
void SimpleRenderer::uniform(const std::string& name, int v1, int v2, int v3) { 
    glUniform3i(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3);
}
void SimpleRenderer::uniform(const std::string& name, int v1, int v2, int v3, int v4) { 
    glUniform4i(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3, v4);
}
void SimpleRenderer::uniform(const std::string& name, unsigned int v1) { 
    glUniform1ui(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1);
}
void SimpleRenderer::uniform(const std::string& name, unsigned int v1, unsigned int v2) { 
    glUniform2ui(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2);
}
void SimpleRenderer::uniform(const std::string& name, unsigned int v1, unsigned int v2, unsigned int v3) { 
    glUniform3ui(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3);
}
void SimpleRenderer::uniform(const std::string& name, unsigned int v1, unsigned int v2, unsigned int v3, unsigned int v4) { 
    glUniform4ui(glGetUniformLocation(impl_->program_.get(), name.c_str()), v1, v2, v3, v4);
}

void SimpleRenderer::render(cv::Mat& dest, const cv::Mat& src)
{
    impl_->render(dest, src);
}