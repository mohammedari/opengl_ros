#ifndef CGS_GL_FRAME_BUFFER_H
#define CGS_GL_FRAME_BUFFER_H

#include <array>
#include <glad/glad.h>

#include "exception.h"
#include "texture2d.h"

namespace cgs {
namespace gl {

class FrameBuffer final
{
    GLuint fbo_;

public:
    FrameBuffer(const Texture2D& texture)
    {
        glCreateFramebuffers(1, &fbo_);
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

} //gl
} //cgs

#endif