#ifndef CGS_GL_SAMPLER_H
#define CGS_GL_SAMPLER_H

#include <glad/glad.h>

#include "exception.h"

namespace cgs {
namespace gl {

class Sampler final
{
    GLuint sampler_;

public:
    Sampler(GLint minFilter, GLint magFilter, GLint warpS, GLint warpT)
    {
        glCreateSamplers(1, &sampler_);
        glSamplerParameteri(sampler_, GL_TEXTURE_MIN_FILTER, minFilter);
        glSamplerParameteri(sampler_, GL_TEXTURE_MAG_FILTER, magFilter);
        glSamplerParameteri(sampler_, GL_TEXTURE_WRAP_S, warpS);
        glSamplerParameteri(sampler_, GL_TEXTURE_WRAP_T, warpT);
    }
    ~Sampler()
    {
        glDeleteSamplers(1, &sampler_);
    }

    void bindToUnit(GLuint unit) const { glBindSampler(unit, sampler_); }

    Sampler(const Sampler&) = delete;
    Sampler& operator=(const Sampler&) = delete;
    Sampler(Sampler&&) = default;
    Sampler& operator=(Sampler&&) = default;
};

} //gl
} //cgs

#endif