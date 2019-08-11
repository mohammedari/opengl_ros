#ifndef CGS_GL_EXCEPTION_H
#define CGS_GL_EXCEPTION_H

#include <stdexcept>
#include <string>

#include <glad/glad.h>

namespace cgs {
namespace gl {

class Exception final : public std::runtime_error
{
public:
    explicit Exception(const std::string& message) : std::runtime_error(message) {}
};

static void handleError(const std::string& message)
{
    throw Exception(message);
}

struct DebugMessageUtil final
{
    static std::string source(GLenum source)
    {
        switch (source)
        {
        case GL_DEBUG_SOURCE_API:
            return "API";
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
            return "WINDOW SYSTEM";
        case GL_DEBUG_SOURCE_SHADER_COMPILER:
            return "SHADER COMPILER";
        case GL_DEBUG_SOURCE_THIRD_PARTY:
            return "THIRD PARTY";
        case GL_DEBUG_SOURCE_APPLICATION:
            return "APPLICATION";
        case GL_DEBUG_SOURCE_OTHER:
            return "OTHER";
        }

        return "UNKNOWN";
    }

    static std::string type(GLenum type)
    {
        switch (type)
        {
        case GL_DEBUG_TYPE_ERROR:
            return "ERROR";
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            return "DEPRECATED_BEHAVIOR";
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            return "UNDEFINED_BEHAVIOR";
        case GL_DEBUG_TYPE_PORTABILITY:
            return "PORTABILITY";
        case GL_DEBUG_TYPE_PERFORMANCE:
            return "PERFORMANCE";
        case GL_DEBUG_TYPE_MARKER:
            return "MARKER";
        case GL_DEBUG_TYPE_OTHER:
            return "OTHER";
        }

        return "UNKNOWN";
    }

    static std::string severity(GLenum severity)
    {
        switch (severity) {
        case GL_DEBUG_SEVERITY_NOTIFICATION:
            return "NOTIFICATION";
        case GL_DEBUG_SEVERITY_LOW:
            return "LOW";
        case GL_DEBUG_SEVERITY_MEDIUM:
            return "MEDIUM";
        case GL_DEBUG_SEVERITY_HIGH:
            return "HIGH";
        }

        return "UNKNOWN";
    }

    DebugMessageUtil() = delete;
    ~DebugMessageUtil() = delete;
    DebugMessageUtil(const DebugMessageUtil&) = delete;
    DebugMessageUtil& operator=(const DebugMessageUtil&) = delete;
    DebugMessageUtil(DebugMessageUtil&&) = delete;
    DebugMessageUtil& operator=(DebugMessageUtil&&) = delete;
};


} //gl
} //cgs

#endif