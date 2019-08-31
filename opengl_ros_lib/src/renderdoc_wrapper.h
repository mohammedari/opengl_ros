#ifndef CGS_RENDERDOC_H
#define CGS_RENDERDOC_H

#include <string>

#include "renderdoc.h"

#include "opengl_context.h"

namespace cgs
{

class RenderDoc final
{
    void* module_ = nullptr;
    RENDERDOC_API_1_4_0* api_ = nullptr; 
    void handleError(const std::string& message, bool ignoreError);

public:
    RenderDoc(const std::string& pathTemplate, bool ignoreError = true);
    ~RenderDoc();

    void StartFrameCapture(const cgs::OpenGLContext& context);
    void EndFrameCapture(const cgs::OpenGLContext& context);

    RenderDoc(const RenderDoc&) = delete;
    RenderDoc& operator=(const RenderDoc&) = delete;
    RenderDoc(RenderDoc&&) = default;
    RenderDoc& operator=(RenderDoc&&) = default;
};

} //cgs

#endif 