#include <array>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <EGL/egl.h>
#include <glad/glad.h>

namespace cgs {
namespace egl  {

class Exception final : public std::runtime_error
{
public:
    Exception(const std::string& message) : std::runtime_error(message) {}
};

static void handleError(const std::string& message)
{
    //TODO ROS Error
    throw Exception(message);
}

static void handleError(EGLBoolean result, const std::string& message)
{
    if (EGL_TRUE == result)
        return;

    std::stringstream ss;
    ss << message << " ";
    switch(result)
    {
    case EGL_FALSE:
        ss << "EGL_FALSE";
        break;
    case EGL_BAD_DISPLAY:
        ss << "EGL_BAD_DISPLAY";
        break;
    case EGL_BAD_PARAMETER:
        ss << "EGL_BAD_PARAMETER";
        break;
    case EGL_BAD_ACCESS:
        ss << "EGL_BAD_ACCESS";
        break;
    case EGL_BAD_MATCH:
        ss << "EGL_BAD_MATCH";
        break;
    case EGL_BAD_SURFACE:
        ss << "EGL_BAD_SURFACE";
        break;
    case EGL_BAD_CONTEXT:
        ss << "EGL_BAD_CONTEXT";
        break;
    case EGL_BAD_NATIVE_WINDOW:
        ss << "EGL_BAD_NATIVE_WINDOW";
        break;
    case EGL_BAD_CURRENT_SURFACE:
        ss << "EGL_BAD_CURRENT_SURFACE";
        break;
    case EGL_CONTEXT_LOST:
        ss << "EGL_CONTEXT_LOST";
        break;
    case EGL_NOT_INITIALIZED:
        ss << "EGL_NOT_INITIALIZED";
        break;
    default:
        ss << "UNKNWON";
    }
    ss << " " << "(" << result << ")";

    handleError(ss.str());
}

class Display final
{
    EGLDisplay display_;
    EGLint versionMajor_;
    EGLint versionMinor_;
    
public:
    explicit Display(decltype(EGL_DEFAULT_DISPLAY) display_id = EGL_DEFAULT_DISPLAY)
    {
        display_ = eglGetDisplay(display_id);
        if (display_ == EGL_NO_DISPLAY) {
            handleError("Failed to get EGL display.");
        }

        handleError(
            eglInitialize(display_, &versionMajor_, &versionMinor_), 
            "Failed to initialize EGL display.");
    }
    ~Display()
    {
        handleError(
            eglTerminate(display_), 
            "Failed to terminate EGL display.");
    }

    EGLDisplay get() const { return display_; }
    EGLint versionMajor() const { return versionMajor_; }
    EGLint versionMinor() const { return versionMinor_; }

    Display(const Display&) = delete;
    Display& operator=(const Display&) = delete;
    Display(Display&&) = default;
    Display& operator=(Display&&) = default;
};

class Context
{
    EGLContext context_;
    const Display& display_;
public:
    explicit Context(const Display& display)
        : display_(display)
    {
        handleError(
            eglBindAPI(EGL_OPENGL_API), 
            "Failed to bind OpenGL API.");

        constexpr std::array<EGLint, 1> configAttributes = {
            EGL_NONE,
        };
        EGLConfig config;
        EGLint numConfig;
        handleError(
            eglChooseConfig(display_.get(), configAttributes.data(), &config, 1, &numConfig), 
            "Failed to choose config");

        //Specifying OpenGL Core 4.5; this should be much glad loader profile.
        const std::array<EGLint, 7> contextAttributes = {
            EGL_CONTEXT_MAJOR_VERSION, 4,
            EGL_CONTEXT_MINOR_VERSION, 5,
            EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE,
        };
        context_ = eglCreateContext(display_.get(), config, EGL_NO_CONTEXT, contextAttributes.data());
        if (context_ == EGL_NO_CONTEXT) {
            handleError("Failed to create EGL context.");
        }

        handleError(
            eglMakeCurrent(display_.get(), EGL_NO_SURFACE, EGL_NO_SURFACE, context_),  
            "Failed to make EGL context current.");
    }
    ~Context()
    {

        handleError(
            eglDestroyContext(display_.get(), context_), 
            "Failed to destroy EGL context.");
    }

    EGLContext get() const { return context_; }

    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = default;
    Context& operator=(Context&&) = default;
};

} //egl
} //cgs

namespace cgs{
namespace gl{

class Exception final : public std::runtime_error
{
public:
    explicit Exception(const std::string& message) : std::runtime_error(message) {}
};

template<class T>
class VertexBuffer
{
    GLuint vbo_;
    const size_t size_;

public:
    template<class CONTAINER>
    VertexBuffer(const CONTAINER& data, GLenum usage)
        : size_(data.size()) 
    {
        glGenBuffers(1, &vbo_);
        glNamedBufferData(vbo_, sizeof(T) * data.size(), data.data(), usage);
    }
    ~VertexBuffer()
    {
        glDeleteBuffers(1, &vbo_);
    }

    void update(const std::vector<T>& data)
    {
        if (data.size() != size_)
            throw Exception("Data size mismatch.");

        auto buffer = glMapNamedBuffer(vbo_, GL_WRITE_ONLY);
        std::copy(data.begin(), data.end(), buffer);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glUnmapNamedBuffer(vbo_);
    }

    GLuint get() const { return vbo_; }

    VertexBuffer(const VertexBuffer&) = delete;
    VertexBuffer& operator=(const VertexBuffer&) = delete;
    VertexBuffer(VertexBuffer&&) = default;
    VertexBuffer& operator=(VertexBuffer&&) = default;
};

class VertexArray
{
    GLuint vao_;

public:
    VertexArray()
    {
        glGenVertexArrays(1, &vao_);
    }
    ~VertexArray()
    {
        glDeleteVertexArrays(1, &vao_);
    }
  
    template<class T>
    void mapVariable(const VertexBuffer<T>& vbo, GLint variable, GLint elementSize, GLenum elementType, GLuint elementOffset)
    {
        glEnableVertexArrayAttrib(vao_, variable);
        glVertexArrayVertexBuffer(vao_, variable, vbo.get(), 0, sizeof(T));
        glVertexArrayAttribFormat(vao_, variable, elementSize, elementType, GL_FALSE, elementOffset);
        glVertexArrayAttribBinding(vao_, variable, variable);
    }
  
    GLuint get() const { return vao_; }
    void bind() const { glBindVertexArray(vao_); }

    VertexArray(const VertexArray&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;
    VertexArray(VertexArray&&) = default;
    VertexArray& operator=(VertexArray&&) = default;
};

class Shader
{
    GLuint shader_;

public:
    Shader(GLenum type, const std::string& filename)
    {
        //reading source code from the file
        std::string source;
        {
            std::ifstream ifs(filename);
            if(ifs.fail())
            {
                std::stringstream ss;
                ss << "Failed to open \"" << filename << "\"";
                throw Exception(ss.str());
            }

            std::copy(
                std::istreambuf_iterator<char>(ifs),
                std::istreambuf_iterator<char>(),
                std::back_inserter(source));
        }

        shader_ = glCreateShader(type);

        if (!shader_)
            throw Exception("Failed to create shader.");

        auto temp = source.c_str();
        glShaderSource(shader_, 1, &temp, nullptr);
        glCompileShader(shader_);

        GLint success;
        glGetShaderiv(shader_, GL_COMPILE_STATUS, &success);
        if (success == GL_FALSE)
        {
            std::array<GLchar, 1024> log;
            glGetShaderInfoLog(shader_, log.size(), nullptr, log.data());
            glDeleteShader(shader_);

            std::stringstream ss;
            ss << "Failed to compile shader. " << log.data();
            throw Exception(ss.str());
        }
    }

    GLuint get() const { return shader_; }

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&&) = default;
    Shader& operator=(Shader&&) = default;
};

class Program
{
    GLuint program_;
public:
    template<class CONTAINER>
    explicit Program(const CONTAINER& shaders)
    {
        program_ = glCreateProgram();

        for (const auto& shader : shaders)
            glAttachShader(program_, shader.get());

        glLinkProgram(program_);

        for (const auto& shader : shaders)
            glDetachShader(program_, shader.get());

        GLint success;
        glGetProgramiv(program_, GL_LINK_STATUS, &success);
        if (success == GL_FALSE)
        {
            std::array<GLchar, 1024> log;
            glGetProgramInfoLog(program_, log.size(), nullptr, log.data());
            glDeleteProgram(program_);

            std::stringstream ss;
            ss << "Failed to link program. " << log.data();
            throw Exception(ss.str());
        }
    }
    ~Program()
    {
        glDeleteProgram(program_);
    }

    GLuint get() const { return program_; }
    void use() const { glUseProgram(program_); }

    Program(const Program&) = delete;
    Program& operator=(const Program&) = delete;
    Program(Program&&) = default;
    Program& operator=(Program&&) = default;
};

class TextureBuffer
{
    GLuint tbo_;
    GLuint texture_;
    int width_, height_, channel_;

public:
    TextureBuffer(int width, int height, int channel, GLenum format, GLenum usage) :
        width_(width), 
        height_(height),
        channel_(channel)
    {
        glGenBuffers(1, &tbo_);
        glNamedBufferData(tbo_, width_ * height_ * channel_, nullptr, usage);

        glGenTextures(1, &texture_);
        glTextureBuffer(texture_, format, tbo_);
    }

    void write(const void* data)
    {
        auto buffer = glMapNamedBuffer(tbo_, GL_WRITE_ONLY);
        memcpy(buffer, data, width_ * height_ * channel_);
        glUnmapNamedBuffer(tbo_);
    }
    void read(void* data)
    {
        const auto buffer = glMapNamedBuffer(tbo_, GL_READ_ONLY);
        memcpy(data, buffer, width_ * height_ * channel_);
        glUnmapNamedBuffer(tbo_);
    }

    GLuint get() const { return texture_; }
    GLsizei width() const { return width_; }
    GLsizei height() const { return height_; }

    TextureBuffer(const TextureBuffer&) = delete;
    TextureBuffer& operator=(const TextureBuffer&) = delete;
    TextureBuffer(TextureBuffer&&) = default;
    TextureBuffer& operator=(TextureBuffer&&) = default;
};

class Texture2D
{
    GLuint texture_;
    GLsizei width_;
    GLsizei height_;

public:
    Texture2D(GLint internalFormat, GLsizei width, GLsizei height) : 
        width_(width), 
        height_(height)
    {
        glGenTextures(1, &texture_);
        glTextureStorage2D(texture_, 0, internalFormat, width_, height_);
    }
    Texture2D(GLint internalFormat, GLsizei width, GLsizei height, GLenum format, GLenum type, const void* data) : 
        Texture2D(internalFormat, width, height)
    {
        write(format, type, data);
    }
    ~Texture2D()
    {
        glDeleteTextures(1, &texture_);
    }

    void write(GLenum format, GLenum type, const void* data)
    {
        glTextureSubImage2D(texture_, 0, 0, 0, width_, height_, format, type, data);
    }
    void read(GLenum format, GLenum type, void* data, GLsizei size)
    {
        glGetTextureImage(texture_, 0, format, type, size, data);
    }

    GLuint get() const { return texture_; }
    GLsizei width() const { return width_; }
    GLsizei height() const { return height_; }

    Texture2D(const Texture2D&) = delete;
    Texture2D& operator=(const Texture2D&) = delete;
    Texture2D(Texture2D&&) = default;
    Texture2D& operator=(Texture2D&&) = default;
};

class FrameBuffer
{
    GLuint fbo_;
public:
    FrameBuffer(const Texture2D& texture)
    {
        glGenFramebuffers(1, &fbo_);
        glNamedFramebufferTexture(fbo_, GL_COLOR_ATTACHMENT0, texture.get(), 0);

        constexpr std::array<GLenum, 1> drawBuffers = {
            GL_COLOR_ATTACHMENT0,
        };
        glNamedFramebufferDrawBuffers(fbo_, drawBuffers.size(), drawBuffers.data());
    }
    ~FrameBuffer()
    {
        glDeleteFramebuffers(1, &fbo_);
    }

    GLuint get() const { return fbo_; }
    void bind() const { glBindFramebuffer(GL_FRAMEBUFFER, fbo_); }

    FrameBuffer(const FrameBuffer&) = delete;
    FrameBuffer& operator=(const FrameBuffer&) = delete;
    FrameBuffer(FrameBuffer&&) = default;
    FrameBuffer& operator=(FrameBuffer&&) = default;
};

} //namespace gl
} //namespace cgs

struct Vertex
{
    float x, y, z;
};

int main()
{
    //Context

    cgs::egl::Display display;
    cgs::egl::Context context(display);

    if(!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(eglGetProcAddress)))
        std::runtime_error("Failed to load OpenGL.");

    std::cout
        << "GL_VENDOR : "                  << glGetString(GL_VENDOR)                   << std::endl
        << "GL_RENDERER : "                << glGetString(GL_RENDERER)                 << std::endl
        << "GL_VERSION : "                 << glGetString(GL_VERSION)                  << std::endl
        << "GL_SHADER_LANGUAGE_VERSION : " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

    glDebugMessageCallback([](
        GLenum source, 
        GLenum type, 
        GLuint id, 
        GLenum severity, 
        GLsizei length, 
        const GLchar* message, 
        const void* userParam
    ) {
        std::cerr << message << std::endl;
    }, nullptr);
    glEnable(GL_DEBUG_OUTPUT);

    //Shader

    std::array<cgs::gl::Shader, 2> shaders = {
        cgs::gl::Shader(GL_VERTEX_SHADER,   "shader/vs_null.glsl"),
        cgs::gl::Shader(GL_FRAGMENT_SHADER, "shader/fs_null.glsl"),
    };
    cgs::gl::Program program(shaders);

    program.use();
    glUniform2f(glGetUniformLocation(program.get(), "resolution"), 640, 480);

    //Vertex

    std::array<Vertex, 3> verticies = {
        Vertex{0, 0, 0}, 
        Vertex{0.3f, 0.3f, 0.3f}, 
        Vertex{0.5f, 0.5f, 0.5f}
    };
    cgs::gl::VertexBuffer<Vertex> vbo(verticies, GL_STATIC_DRAW);

    cgs::gl::VertexArray vao;
    vao.mapVariable(vbo, glGetAttribLocation(program.get(), "position"), 3, GL_FLOAT, 0);

    //Texture

    constexpr auto width = 200;
    constexpr auto height = 100;

    cgs::gl::Texture2D texture(GL_RGB, width, height);
    cgs::gl::FrameBuffer fbo(texture);

    //Render

    fbo.bind();
    glViewport(0, 0, width, height);
    glClearColor(0, 1.0f, 0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    vao.bind();
    glDrawArrays(GL_POINTS, 0, verticies.size());

    glFinish();

    //Save

    cv::Mat image(height, width, CV_8UC3);
    texture.read(GL_BGR, GL_UNSIGNED_BYTE, image.data, image.rows * image.cols * image.channels());

    cv::imwrite("output.png", image);

    return 0;
}