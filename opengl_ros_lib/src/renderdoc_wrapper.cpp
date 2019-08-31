#include "renderdoc_wrapper.h"

#include <stdexcept>
#include <dlfcn.h>

#include <ros/ros.h>

using namespace cgs;

void RenderDoc::handleError(const std::string& message, bool ignoreError)
{
    if (ignoreError)
    {
        ROS_WARN_STREAM(message);
        return;
    }

    ROS_FATAL_STREAM(message);
    throw std::runtime_error(message);
}

RenderDoc::RenderDoc(const std::string& pathTemplate, bool ignoreError)
{
    module_ = dlopen("librenderdoc.so", RTLD_NOW | RTLD_NOLOAD);
    if (!module_)
    {
        handleError("Failed to load librenderdoc.so.", ignoreError);
        return;
    }

    auto getapi = reinterpret_cast<pRENDERDOC_GetAPI>(dlsym(module_, "RENDERDOC_GetAPI"));
    if (!getapi)
    {
        handleError("Failed to find RENDERDOC_GetAPI entry point in the module.", ignoreError);
        return;
    }

    auto result = getapi(eRENDERDOC_API_Version_1_4_0, reinterpret_cast<void **>(&api_));
    if (result != 1)
    {
        handleError("RENDERDOC_GetAPI method failed.", ignoreError);
        return;
    }

    if (api_) api_->SetCaptureFilePathTemplate(pathTemplate.c_str());
}

RenderDoc::~RenderDoc()
{
    if (api_) api_->Shutdown();
    if (module_) dlclose(module_);
}

void RenderDoc::StartFrameCapture(const OpenGLContext& context)
{
    if (api_) api_->StartFrameCapture(context.get(), nullptr);
}

void RenderDoc::EndFrameCapture(const OpenGLContext& context)
{
    if (api_) api_->EndFrameCapture(context.get(), nullptr);
}