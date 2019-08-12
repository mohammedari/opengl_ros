#ifndef CGS_GL_SHADER_H
#define CGS_GL_SHADER_H

#include <array>
#include <fstream>
#include <sstream>

#include <glad/glad.h>

#include "exception.h"

namespace cgs {
namespace gl {

class Shader
{
    GLuint shader_;

public:
    Shader(GLenum type, const std::string& filename)
    {
        //reading source code from the file
        std::string source;
        {
            std::ifstream ifs(filename);
            if(ifs.fail())
            {
                std::stringstream ss;
                ss << "Failed to initialize shader. Failed to open \"" << filename << "\"";
                handleError(ss.str());
            }

            std::copy(
                std::istreambuf_iterator<char>(ifs),
                std::istreambuf_iterator<char>(),
                std::back_inserter(source));
        }

        shader_ = glCreateShader(type);

        if (!shader_)
            throw Exception("Failed to create shader.");

        auto temp = source.c_str();
        glShaderSource(shader_, 1, &temp, nullptr);
        glCompileShader(shader_);

        GLint success;
        glGetShaderiv(shader_, GL_COMPILE_STATUS, &success);
        if (success == GL_FALSE)
        {
            std::array<GLchar, 1024> log;
            glGetShaderInfoLog(shader_, log.size(), nullptr, log.data());
            glDeleteShader(shader_);

            std::stringstream ss;
            ss << "Failed to compile shader. " << log.data();
            handleError(ss.str());
        }
    }

    GLuint get() const { return shader_; }

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&&) = default;
    Shader& operator=(Shader&&) = default;
};

} //gl
} //cgs

#endif