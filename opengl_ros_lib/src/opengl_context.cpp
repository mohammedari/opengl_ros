#include "opengl_context.h"

#include "gl/gl.h"

using namespace cgs;

OpenGLContext::OpenGLContext(bool enableDebugOutput) :
    display_(), context_(display_) 
{
    if(!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(eglGetProcAddress)))
    {
        ROS_FATAL_STREAM("Failed to load OpenGL.");
        std::runtime_error("Failed to load OpenGL.");
    }

    ROS_INFO_STREAM(
        "OpenGL successfully initialized." << std::endl <<
        "GL_VENDOR : "                     << glGetString(GL_VENDOR)                   << std::endl << 
        "GL_RENDERER : "                   << glGetString(GL_RENDERER)                 << std::endl <<
        "GL_VERSION : "                    << glGetString(GL_VERSION)                  << std::endl <<
        "GL_SHADER_LANGUAGE_VERSION : "    << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
    );

    glDebugMessageCallback([](
        GLenum source, 
        GLenum type, 
        GLuint id, 
        GLenum severity, 
        GLsizei length, 
        const GLchar* message, 
        const void* userParam
    ) {
        std::stringstream ss;
        ss << cgs::gl::DebugMessageUtil::source(source)     << ", "
           << cgs::gl::DebugMessageUtil::type(type)         << ", "
           << cgs::gl::DebugMessageUtil::severity(severity) << ", "  
           << id << ": " 
           << message << std::endl;

        switch (severity) {
        case GL_DEBUG_SEVERITY_NOTIFICATION:
            ROS_DEBUG_STREAM(ss.str());
            break;
        case GL_DEBUG_SEVERITY_LOW:
            ROS_INFO_STREAM(ss.str());
            break;
        case GL_DEBUG_SEVERITY_MEDIUM:
            ROS_WARN_STREAM(ss.str());
            break;
        case GL_DEBUG_SEVERITY_HIGH:
            ROS_ERROR_STREAM(ss.str());
            break;
        default:
            ROS_INFO_STREAM(ss.str());
        }
    }, nullptr);

    if (enableDebugOutput)
        glEnable(GL_DEBUG_OUTPUT);
}