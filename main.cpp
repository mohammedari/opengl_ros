#include <array>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>

#include <EGL/egl.h>
#include <GL/gl.h>

namespace cgs {
namespace gl  {

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

    std::stringstream ss(message);
    ss << " ";
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
    Display(decltype(EGL_DEFAULT_DISPLAY) display_id = EGL_DEFAULT_DISPLAY)
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
};

class Context
{
    EGLContext context_;
    const Display& display_;
public:
    Context(const Display& display)
        : display_(display)
    {
        handleError(
            eglBindAPI(EGL_OPENGL_API), 
            "Failed to bind OpenGL API.");

        constexpr std::array<EGLint, 1> attributes = {
            EGL_NONE,
        };
        EGLConfig config;
        EGLint numConfig;
        handleError(
            eglChooseConfig(display_.get(), attributes.data(), &config, 1, &numConfig), 
            "Failed to choose config");

        context_ = eglCreateContext(display_.get(), config, EGL_NO_CONTEXT, nullptr);
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
};

class Renderer
{

public:
    Renderer()
    {

    }
    ~Renderer()
    {

    }
};

} //namespace cgs
} //namespace cgs

int main()
{
    auto display = cgs::gl::Display();
    auto context = cgs::gl::Context(display);

    std::cout
        << "GL_VENDOR : "                  << glGetString(GL_VENDOR)                   << std::endl
        << "GL_RENDERER : "                << glGetString(GL_RENDERER)                 << std::endl
        << "GL_VERSION : "                 << glGetString(GL_VERSION)                  << std::endl
        << "GL_SHADER_LANGUAGE_VERSION : " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl
        << std::endl;

    return 0;
}