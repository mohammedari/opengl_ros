#ifndef CGS_GL_PROGRAM_H
#define CGS_GL_PROGRAM_H

#include <glad/glad.h>

#include "exception.h"

namespace cgs {
namespace gl {

class Program final
{
    GLuint program_;

public:
    template<class CONTAINER>
    explicit Program(const CONTAINER& shaders)
    {
        program_ = glCreateProgram();

        for (const auto& shader : shaders)
            glAttachShader(program_, shader.get());

        glLinkProgram(program_);

        for (const auto& shader : shaders)
            glDetachShader(program_, shader.get());

        GLint success;
        glGetProgramiv(program_, GL_LINK_STATUS, &success);
        if (success == GL_FALSE)
        {
            std::array<GLchar, 1024> log;
            glGetProgramInfoLog(program_, log.size(), nullptr, log.data());
            glDeleteProgram(program_);

            std::stringstream ss;
            ss << "Failed to link program. " << log.data();
            throw Exception(ss.str());
        }
    }
    ~Program()
    {
        glDeleteProgram(program_);
    }

    GLuint get() const { return program_; }
    void use() const { glUseProgram(program_); }

    Program(const Program&) = delete;
    Program& operator=(const Program&) = delete;
    Program(Program&&) = default;
    Program& operator=(Program&&) = default;
};

} //gl
} //cgs

#endif