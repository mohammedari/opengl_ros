#ifndef CGS_EGL_CONTEXT_H
#define CGS_EGL_CONTEXT_H

#include <array>

#include <EGL/egl.h>

#include "display.h"
#include "exception.h"

namespace cgs {
namespace egl {

class Context final
{
    EGLContext context_;
    const Display& display_;
public:
    explicit Context(const Display& display)
        : display_(display)
    {
        handleError(
            eglBindAPI(EGL_OPENGL_API),·
            "Failed to bind OpenGL API.");

        constexpr std::array<EGLint, 1> configAttributes = {
            EGL_NONE,
        };
        EGLConfig config;
        EGLint numConfig;
        handleError(
            eglChooseConfig(display_.get(), configAttributes.data(), &config, 1, &numConfig),·
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
            eglMakeCurrent(display_.get(), EGL_NO_SURFACE, EGL_NO_SURFACE, context_),··
            "Failed to make EGL context current.");
    }
    ~Context()
    {

        handleError(
            eglDestroyContext(display_.get(), context_),·
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

#endif
