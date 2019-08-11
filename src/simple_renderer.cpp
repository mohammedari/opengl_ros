
#include "simple_renderer.h"

#include <array>

#include "egl/egl.h"
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

    struct Egl
    {
        cgs::egl::Display display_;
        cgs::egl::Context context_;
        Egl();
    };

    Egl egl_;
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

SimpleRenderer::Impl::Egl::Egl() :
    display_(), context_(display_) 
{
    //TODO use ROS errors
    if(!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(eglGetProcAddress)))
        std::runtime_error("Failed to load OpenGL.");

    //TODO use ROS output
    std::cout
        << "OpenGL successfully initialized." << std::endl
        << "GL_VENDOR : "                     << glGetString(GL_VENDOR)                   << std::endl
        << "GL_RENDERER : "                   << glGetString(GL_RENDERER)                 << std::endl
        << "GL_VERSION : "                    << glGetString(GL_VERSION)                  << std::endl
        << "GL_SHADER_LANGUAGE_VERSION : "    << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

    glDebugMessageCallback([](
        GLenum source, 
        GLenum type, 
        GLuint id, 
        GLenum severity, 
        GLsizei length, 
        const GLchar* message, 
        const void* userParam
    ) {
        //TODO use ROS errors
        std::cerr 
            << cgs::gl::DebugMessageUtil::source(source)     << ", " 
            << cgs::gl::DebugMessageUtil::type(type)         << ", " 
            << cgs::gl::DebugMessageUtil::severity(severity) << ", " 
            << id << ": " 
            << message << std::endl;
    }, nullptr);
    glEnable(GL_DEBUG_OUTPUT);
}

SimpleRenderer::Impl::Impl(
    int width, int height, 
    const std::string& vertexShader, 
    const std::string& fragmentShader) : 
    width_(width), height_(height), 
    shaders_({
        cgs::gl::Shader(GL_VERTEX_SHADER,   "shader/vs_passthrough.glsl"),
        cgs::gl::Shader(GL_FRAGMENT_SHADER, "shader/fs_passthrough.glsl"),
    }), 
    program_(shaders_), 
    vbo_(VERTICIES, GL_STATIC_DRAW), 
    ebo_(INDICIES, GL_STATIC_DRAW), 
    textureIn_(GL_RGB8, width_, height_),
    textureOut_(GL_RGB8, width_, height_),
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
        //TODO use ROS errors
        throw std::runtime_error("Destination image resolution does not match.");
    }

    if (width_ != src.cols || height_ != src.rows || CV_8UC3 != dest.type())
    {
        //TODO use ROS errors
        throw std::runtime_error("Source image resolution does not match.");
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
    //TODO use ROS errors
    throw;
}
catch (cgs::gl::Exception& e)
{
    //TODO use ROS errors
    throw;
}

SimpleRenderer::~SimpleRenderer() = default;

void SimpleRenderer::render(cv::Mat& dest, const cv::Mat& src)
{
    impl_->render(dest, src);
}