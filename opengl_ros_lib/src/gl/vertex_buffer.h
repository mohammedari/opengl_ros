#ifndef CGS_GL_VERTEX_BUFFER_H
#define CGS_GL_VERTEX_BUFFER_H

#include <glad/glad.h>

#include "exception.h"

namespace cgs {
namespace gl {

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
        glCreateBuffers(1, &vbo_);
        glNamedBufferData(vbo_, sizeof(T) * data.size(), data.data(), usage);
    }
    ~VertexBuffer()
    {
        glDeleteBuffers(1, &vbo_);
    }

    template<class CONTAINER>
    void update(const CONTAINER& data)
    {
        if (data.size() != size_)
            handleError("Data size mismatch.");

        auto buffer = glMapNamedBuffer(vbo_, GL_WRITE_ONLY);
        std::copy(data.begin(), data.end(), buffer);
        glUnmapNamedBuffer(vbo_);
    }

    GLuint get() const { return vbo_; }

    VertexBuffer(const VertexBuffer&) = delete;
    VertexBuffer& operator=(const VertexBuffer&) = delete;
    VertexBuffer(VertexBuffer&&) = default;
    VertexBuffer& operator=(VertexBuffer&&) = default;
};

} //gl
} //cgs

#endif