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
            eglBindAPI(EGL_OPENGL_API), 
            "Failed to bind OpenGL API.");

	static const EGLint configAttributes[] = {
	        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
	        EGL_BLUE_SIZE, 8,
	        EGL_GREEN_SIZE, 8,
	        EGL_RED_SIZE, 8,
	        EGL_DEPTH_SIZE, 8,
	        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
	        EGL_NONE
	};    

        EGLConfig config;
        EGLint numConfig;
        handleError(
            eglChooseConfig(display_.get(), configAttributes, &config, 1, &numConfig), 
            "Failed to choose config");

        //Specifying OpenGL Core 4.5; this should be much glad loader profile.
        const std::array<EGLint, 7> contextAttributes = {
            EGL_CONTEXT_MAJOR_VERSION, 4,
            EGL_CONTEXT_MINOR_VERSION, 5,
            EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE,
        };

  	static const int pbufferWidth = 9;
  	static const int pbufferHeight = 9;

  	static const EGLint pbufferAttribs[] = {
  	      EGL_WIDTH, pbufferWidth,
  	      EGL_HEIGHT, pbufferHeight,
  	      EGL_NONE,
  	};
	EGLSurface eglSurf = eglCreatePbufferSurface(display_.get(), &config, pbufferAttribs);

        context_ = eglCreateContext(display_.get(), config, EGL_NO_CONTEXT, NULL);
        if (context_ == EGL_NO_CONTEXT) {
            handleError("Failed to create EGL context.");
        }

        handleError(
            eglMakeCurrent(display_.get(), eglSurf, eglSurf, context_),  
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

#endif
