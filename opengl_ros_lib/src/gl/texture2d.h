#ifndef CGS_GL_TEXTURE2D_H
#define CGS_GL_TEXTURE2D_H

#include <glad/glad.h>

#include "exception.h"

namespace cgs {
namespace gl {

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
        glCreateTextures(GL_TEXTURE_2D, 1, &texture_);
        glTextureStorage2D(texture_, 1, internalFormat, width_, height_);
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

    void bindToUnit(GLuint unit) const { glBindTextureUnit(unit, texture_); }

    Texture2D(const Texture2D&) = delete;
    Texture2D& operator=(const Texture2D&) = delete;
    Texture2D(Texture2D&&) = default;
    Texture2D& operator=(Texture2D&&) = default;
};

} //gl
} //cgs

#endif