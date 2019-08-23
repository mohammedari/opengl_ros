#ifndef CGS_GL_TEXTURE_BUFFER_H
#define CGS_GL_TEXTURE_BUFFER_H

#include <cstring>
#include <glad/glad.h>

#include "exception.h"

namespace cgs {
namespace gl {

class TextureBuffer final
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
        glCreateBuffers(1, &tbo_);
        glNamedBufferData(tbo_, width_ * height_ * channel_, nullptr, usage);

        glCreateTextures(GL_TEXTURE_BUFFER, 1, &texture_);
        glTextureBuffer(texture_, format, tbo_);
    }

    void write(const void* data)
    {
        auto buffer = glMapNamedBuffer(tbo_, GL_WRITE_ONLY);
        std::memcpy(buffer, data, width_ * height_ * channel_);
        glUnmapNamedBuffer(tbo_);
    }
    void read(void* data)
    {
        const auto buffer = glMapNamedBuffer(tbo_, GL_READ_ONLY);
        std::memcpy(data, buffer, width_ * height_ * channel_);
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

} //gl
} //cgs

#endif