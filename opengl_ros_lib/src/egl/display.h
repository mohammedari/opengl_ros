#ifndef CGS_EGL_DISPLAY_H
#define CGS_EGL_DISPLAY_H

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "exception.h"

namespace cgs {
namespace egl {

class Display final
{
    EGLDisplay display_;
    EGLint versionMajor_;
    EGLint versionMinor_;
    
public:
    explicit Display(decltype(EGL_DEFAULT_DISPLAY) display_id = EGL_DEFAULT_DISPLAY)
    {
        //display_ = eglGetDisplay(display_id);
	static const int MAX_DEVICES = 4;
	EGLDeviceEXT eglDevs[MAX_DEVICES];
	EGLint numDevices;
	
	PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT = (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
	eglQueryDevicesEXT(MAX_DEVICES, eglDevs, &numDevices);
	
	printf("Detected %d devices\n", numDevices);
	
	PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT = (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
	EGLDisplay display_ = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, eglDevs[0], 0);  
        if (display_ == EGL_NO_DISPLAY) {
            handleError("Failed to get EGL display.");
        }

        handleError(
            eglInitialize(display_, &versionMajor_, &versionMinor_), 
            "Failed to initialize EGL display.");
    }
    ~Display()
    {
        handleError(
            eglTerminate(display_), 
            "Failed to terminate EGL display.");
    }

    EGLDisplay get() const { return display_; }
    EGLint versionMajor() const { return versionMajor_; }
    EGLint versionMinor() const { return versionMinor_; }

    Display(const Display&) = delete;
    Display& operator=(const Display&) = delete;
    Display(Display&&) = default;
    Display& operator=(Display&&) = default;
};

} //egl
} //cgs

#endif
