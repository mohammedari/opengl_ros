#ifndef CGS_GL_VERTEX_ARRAY_H
#define CGS_GL_VERTEX_ARRAY_H

#include <glad/glad.h>

#include "exception.h"
#include "vertex_buffer.h"
#include "element_buffer.h"

namespace cgs {
namespace gl {

class VertexArray
{
    GLuint vao_;

public:
    VertexArray()
    {
        glCreateVertexArrays(1, &vao_);
    }
    ~VertexArray()
    {
        glDeleteVertexArrays(1, &vao_);
    }
  
    template<class T>
    void mapVariable(const VertexBuffer<T>& vbo, GLint variable, GLint elementSize, GLenum elementType, GLuint elementOffset)
    {
        glEnableVertexArrayAttrib(vao_, variable);
        glVertexArrayVertexBuffer(vao_, variable, vbo.get(), 0, sizeof(T));
        glVertexArrayAttribFormat(vao_, variable, elementSize, elementType, GL_FALSE, elementOffset);
        glVertexArrayAttribBinding(vao_, variable, variable);
    }

    template<class T>
    void mapVariable(const ElementBuffer<T>& ebo)
    {
        glVertexArrayElementBuffer(vao_, ebo.get());
    }
  
    GLuint get() const { return vao_; }
    void bind() const { glBindVertexArray(vao_); }

    VertexArray(const VertexArray&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;
    VertexArray(VertexArray&&) = default;
    VertexArray& operator=(VertexArray&&) = default;
};

} //gl
} //cgs

#endif