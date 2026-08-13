#pragma once

#define XR_USE_GRAPHICS_API_OPENGL_ES
#define XR_USE_PLATFORM_ANDROID

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <android_native_app_glue.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <vector>

#include "gl_renderer.h"
#include "pov_fetcher.h"

// Owns the whole OpenXR + EGL + swapchain lifecycle and per-frame render loop. v1 scope:
// display-only -- renders the stereo POV feed (see PovFetcher) into both eyes every frame.
// Hand tracking (XR_EXT_hand_tracking) and sending wrist poses back to quest_teleop_node is
// NOT implemented yet (see README.md) -- this replaces index.html's *display* path only.
class XrAppState {
public:
    // android_app->userData is set to this instance; onAppCmd/onInputEvent are wired to the
    // static trampolines below, matching the android_native_app_glue callback signature.
    void run(android_app *app);

private:
    bool initInstanceAndSystem();
    bool initEgl();
    bool initSession();
    bool initSpaces();
    bool initSwapchains();
    void pollXrEvents(android_app *app);
    void renderFrame();
    void renderEyeToSwapchain(uint32_t eyeIndex, XrSwapchainSubImage &outSubImage);
    void shutdown();

    static void onAppCmd(android_app *app, int32_t cmd);

    android_app *app_ = nullptr;

    XrInstance instance_ = XR_NULL_HANDLE;
    XrSystemId systemId_ = XR_NULL_SYSTEM_ID;
    XrSession session_ = XR_NULL_HANDLE;
    XrSpace localSpace_ = XR_NULL_HANDLE;
    XrSessionState sessionState_ = XR_SESSION_STATE_UNKNOWN;
    bool sessionRunning_ = false;
    bool requestedExit_ = false;

    EGLDisplay eglDisplay_ = EGL_NO_DISPLAY;
    EGLContext eglContext_ = EGL_NO_CONTEXT;
    EGLConfig eglConfig_ = nullptr;
    EGLSurface eglSurface_ = EGL_NO_SURFACE;  // tiny offscreen pbuffer -- the XR compositor owns the real output

    struct Swapchain {
        XrSwapchain handle = XR_NULL_HANDLE;
        int32_t width = 0;
        int32_t height = 0;
        std::vector<XrSwapchainImageOpenGLESKHR> images;
    };
    Swapchain swapchains_[2];  // 0=left, 1=right, matching XrViewConfigurationView order for STEREO
    std::vector<XrViewConfigurationView> viewConfigViews_;
    GLuint fbo_ = 0;

    GlRenderer eyeRenderers_[2];
    PovFetcher povFetcher_;
};
