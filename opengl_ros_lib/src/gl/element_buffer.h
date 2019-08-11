#ifndef CGS_GL_ELEMENT_BUFFER_H
#define CGS_GL_ELEMENT_BUFFER_H

#include <glad/glad.h>

#include "exception.h"

namespace cgs {
namespace gl {

template<class T>
class ElementBuffer
{
    GLuint ebo_;
    const size_t size_;

public:
    template<class CONTAINER>
    ElementBuffer(const CONTAINER& data, GLenum usage)
        : size_(data.size()) 
    {
        glCreateBuffers(1, &ebo_);
        glNamedBufferData(ebo_, sizeof(T) * data.size(), data.data(), usage);
    }
    ~ElementBuffer()
    {
        glDeleteBuffers(1, &ebo_);
    }

    template<class CONTAINER>
    void update(const CONTAINER& data)
    {
        if (data.size() != size_)
            handleError("Data size mismatch.");

        auto buffer = glMapNamedBuffer(ebo_, GL_WRITE_ONLY);
        std::copy(data.begin(), data.end(), buffer);
        glUnmapNamedBuffer(ebo_);
    }

    GLuint get() const { return ebo_; }

    ElementBuffer(const ElementBuffer&) = delete;
    ElementBuffer& operator=(const ElementBuffer&) = delete;
    ElementBuffer(ElementBuffer&&) = default;
    ElementBuffer& operator=(ElementBuffer&&) = default;
};

} //gl
} //cgs

#endif