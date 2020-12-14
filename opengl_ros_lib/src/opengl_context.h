#ifndef CGS_OPENGL_CONTEXT_H
#define CGS_OPENGL_CONTEXT_H

#include <ros/ros.h>

#include "egl/egl.h"

namespace cgs {

class OpenGLContext
{
    cgs::egl::Display display_;
    cgs::egl::Context context_;

public:
    explicit OpenGLContext(bool enableDebugOutput);

    OpenGLContext(const OpenGLContext&) = delete;
    OpenGLContext& operator=(const OpenGLContext&) = delete;
    OpenGLContext(OpenGLContext&&) = default;
    OpenGLContext& operator=(OpenGLContext&&) = default;
};

} //cgs

#endif