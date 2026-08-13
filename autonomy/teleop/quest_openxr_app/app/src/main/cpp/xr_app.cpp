#include "xr_app.h"

#include <android/log.h>

#include <cstring>
#include <vector>

#define LOG_TAG "questopenxr.xr"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)

// Host running webxr_server.py -- 127.0.0.1 matches this project's already-established workflow
// of `adb reverse tcp:8080 tcp:8080` (same pattern already used for tcp:8443/tcp:9090 with the
// browser client). If testing over plain Wi-Fi instead of USB/adb reverse, change this to the
// dev machine's LAN IP.
static const char *kPovHost = "127.0.0.1";

namespace {

bool xrCheck(XrResult result, const char *what) {
    if (XR_FAILED(result)) {
        LOGE("%s failed: XrResult=%d", what, static_cast<int>(result));
        return false;
    }
    return true;
}

}  // namespace

void XrAppState::onAppCmd(android_app *app, int32_t cmd) {
    auto *self = static_cast<XrAppState *>(app->userData);
    switch (cmd) {
        case APP_CMD_INIT_WINDOW:
            LOGI("APP_CMD_INIT_WINDOW");
            // Everything OpenXR-side is initialized here, once, the first time Android gives us
            // a native window -- mirrors hello_xr/Meta sample structure. On Quest the whole
            // lifecycle is headset-driven (there's no real "window" the way a phone app has one),
            // but native_app_glue still requires a window callback before assuming a valid
            // rendering context can exist.
            if (self->instance_ == XR_NULL_HANDLE) {
                if (!self->initInstanceAndSystem()) { LOGE("initInstanceAndSystem failed"); return; }
                if (!self->initEgl()) { LOGE("initEgl failed"); return; }
                if (!self->initSession()) { LOGE("initSession failed"); return; }
                if (!self->initSpaces()) { LOGE("initSpaces failed"); return; }
                if (!self->initSwapchains()) { LOGE("initSwapchains failed"); return; }
                self->povFetcher_.start(kPovHost);
            }
            break;
        case APP_CMD_TERM_WINDOW:
            LOGI("APP_CMD_TERM_WINDOW");
            break;
        case APP_CMD_DESTROY:
            LOGI("APP_CMD_DESTROY");
            self->requestedExit_ = true;
            break;
        default:
            break;
    }
}

bool XrAppState::initInstanceAndSystem() {
    // Android's OpenXR loader needs the JavaVM + Activity to locate the runtime broker before
    // any other OpenXR call -- easy to forget, and xrCreateInstance fails opaquely without it.
    PFN_xrInitializeLoaderKHR xrInitializeLoaderKHR = nullptr;
    xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                           reinterpret_cast<PFN_xrVoidFunction *>(&xrInitializeLoaderKHR));
    if (xrInitializeLoaderKHR != nullptr) {
        XrLoaderInitInfoAndroidKHR loaderInitInfo{XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
        loaderInitInfo.applicationVM = app_->activity->vm;
        loaderInitInfo.applicationContext = app_->activity->clazz;
        xrInitializeLoaderKHR(reinterpret_cast<XrLoaderInitInfoBaseHeaderKHR *>(&loaderInitInfo));
    } else {
        LOGW("xrInitializeLoaderKHR not available -- proceeding anyway (some runtimes don't need it)");
    }

    const char *extensions[] = {
        XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,
        XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME,
    };

    XrInstanceCreateInfoAndroidKHR androidInfo{XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
    androidInfo.applicationVM = app_->activity->vm;
    androidInfo.applicationActivity = app_->activity->clazz;

    XrInstanceCreateInfo createInfo{XR_TYPE_INSTANCE_CREATE_INFO};
    createInfo.next = &androidInfo;
    createInfo.enabledExtensionCount = 2;
    createInfo.enabledExtensionNames = extensions;
    std::strncpy(createInfo.applicationInfo.applicationName, "quest_openxr_app",
                 XR_MAX_APPLICATION_NAME_SIZE - 1);
    createInfo.applicationInfo.applicationVersion = 1;
    createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

    if (!xrCheck(xrCreateInstance(&createInfo, &instance_), "xrCreateInstance")) return false;

    XrSystemGetInfo systemInfo{XR_TYPE_SYSTEM_GET_INFO};
    systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    if (!xrCheck(xrGetSystem(instance_, &systemInfo, &systemId_), "xrGetSystem")) return false;

    LOGI("OpenXR instance + system created OK (systemId=%llu)",
         static_cast<unsigned long long>(systemId_));
    return true;
}

bool XrAppState::initEgl() {
    eglDisplay_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (eglDisplay_ == EGL_NO_DISPLAY) { LOGE("eglGetDisplay failed"); return false; }
    if (eglInitialize(eglDisplay_, nullptr, nullptr) == EGL_FALSE) {
        LOGE("eglInitialize failed");
        return false;
    }

    const EGLint configAttribs[] = {
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_NONE,
    };
    EGLint numConfigs = 0;
    if (eglChooseConfig(eglDisplay_, configAttribs, &eglConfig_, 1, &numConfigs) == EGL_FALSE
        || numConfigs == 0) {
        LOGE("eglChooseConfig failed");
        return false;
    }

    const EGLint contextAttribs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
    eglContext_ = eglCreateContext(eglDisplay_, eglConfig_, EGL_NO_CONTEXT, contextAttribs);
    if (eglContext_ == EGL_NO_CONTEXT) { LOGE("eglCreateContext failed"); return false; }

    // A tiny offscreen pbuffer -- eglMakeCurrent requires *some* surface, but actual rendering
    // targets the swapchain-image-backed FBOs created in initSwapchains/renderEyeToSwapchain,
    // not this surface (the OpenXR compositor owns final display output on Quest, unlike a
    // desktop GL app that presents to a window).
    const EGLint pbufferAttribs[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    eglSurface_ = eglCreatePbufferSurface(eglDisplay_, eglConfig_, pbufferAttribs);
    if (eglSurface_ == EGL_NO_SURFACE) { LOGE("eglCreatePbufferSurface failed"); return false; }

    if (eglMakeCurrent(eglDisplay_, eglSurface_, eglSurface_, eglContext_) == EGL_FALSE) {
        LOGE("eglMakeCurrent failed");
        return false;
    }

    // Spec requires querying graphics requirements before session creation, even though we
    // don't gate GL ES version selection on the result here (Quest's runtime always supports
    // GLES 3.x, which is what we requested above).
    PFN_xrGetOpenGLESGraphicsRequirementsKHR xrGetOpenGLESGraphicsRequirementsKHR = nullptr;
    xrGetInstanceProcAddr(instance_, "xrGetOpenGLESGraphicsRequirementsKHR",
                           reinterpret_cast<PFN_xrVoidFunction *>(&xrGetOpenGLESGraphicsRequirementsKHR));
    if (xrGetOpenGLESGraphicsRequirementsKHR != nullptr) {
        XrGraphicsRequirementsOpenGLESKHR reqs{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
        xrGetOpenGLESGraphicsRequirementsKHR(instance_, systemId_, &reqs);
        LOGI("GLES requirements: min=%u.%u max=%u.%u",
             static_cast<unsigned>(XR_VERSION_MAJOR(reqs.minApiVersionSupported)),
             static_cast<unsigned>(XR_VERSION_MINOR(reqs.minApiVersionSupported)),
             static_cast<unsigned>(XR_VERSION_MAJOR(reqs.maxApiVersionSupported)),
             static_cast<unsigned>(XR_VERSION_MINOR(reqs.maxApiVersionSupported)));
    }

    LOGI("EGL initialized OK");
    return true;
}

bool XrAppState::initSession() {
    XrGraphicsBindingOpenGLESAndroidKHR graphicsBinding{XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
    graphicsBinding.display = eglDisplay_;
    graphicsBinding.config = eglConfig_;
    graphicsBinding.context = eglContext_;

    XrSessionCreateInfo createInfo{XR_TYPE_SESSION_CREATE_INFO};
    createInfo.next = &graphicsBinding;
    createInfo.systemId = systemId_;
    if (!xrCheck(xrCreateSession(instance_, &createInfo, &session_), "xrCreateSession")) return false;

    LOGI("OpenXR session created OK");
    return true;
}

bool XrAppState::initSpaces() {
    XrReferenceSpaceCreateInfo spaceInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    spaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    spaceInfo.poseInReferenceSpace.orientation.w = 1.0f;
    if (!xrCheck(xrCreateReferenceSpace(session_, &spaceInfo, &localSpace_), "xrCreateReferenceSpace"))
        return false;
    return true;
}

bool XrAppState::initSwapchains() {
    uint32_t viewCount = 0;
    xrEnumerateViewConfigurationViews(instance_, systemId_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                       0, &viewCount, nullptr);
    if (viewCount != 2) {
        LOGE("Expected stereo (2 views), got %u", viewCount);
        return false;
    }
    viewConfigViews_.resize(viewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
    xrEnumerateViewConfigurationViews(instance_, systemId_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                       viewCount, &viewCount, viewConfigViews_.data());

    uint32_t formatCount = 0;
    xrEnumerateSwapchainFormats(session_, 0, &formatCount, nullptr);
    std::vector<int64_t> formats(formatCount);
    xrEnumerateSwapchainFormats(session_, formatCount, &formatCount, formats.data());
    // GL_RGBA8 (0x8058) is the standard, always-safe choice; fall back to whatever the runtime
    // lists first if it's somehow not offered.
    int64_t chosenFormat = formats.empty() ? 0x8058 : formats[0];
    for (int64_t f : formats) {
        if (f == 0x8058 /* GL_RGBA8 */) { chosenFormat = f; break; }
    }

    for (uint32_t i = 0; i < 2; ++i) {
        Swapchain &sc = swapchains_[i];
        // Matches pov_left.png/pov_right.png's actual resolution (confirmed via direct
        // inspection: 1280x720), NOT viewConfigViews_'s HMD-recommended per-eye resolution
        // (1440x1584) -- these are now XrCompositionLayerQuad panels (see renderFrame), not
        // XrCompositionLayerProjection eye buffers, so there's no reason to render at a larger
        // size than the source content actually has; the quad's own size (in meters) is what
        // determines on-screen scale, independent of swapchain pixel dimensions.
        sc.width = 1280;
        sc.height = 720;

        XrSwapchainCreateInfo scInfo{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        scInfo.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
        scInfo.format = chosenFormat;
        scInfo.sampleCount = 1;
        scInfo.width = sc.width;
        scInfo.height = sc.height;
        scInfo.faceCount = 1;
        scInfo.arraySize = 1;
        scInfo.mipCount = 1;
        if (!xrCheck(xrCreateSwapchain(session_, &scInfo, &sc.handle), "xrCreateSwapchain")) return false;

        uint32_t imageCount = 0;
        xrEnumerateSwapchainImages(sc.handle, 0, &imageCount, nullptr);
        sc.images.resize(imageCount, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR});
        xrEnumerateSwapchainImages(
            sc.handle, imageCount, &imageCount,
            reinterpret_cast<XrSwapchainImageBaseHeader *>(sc.images.data()));

        LOGI("Eye %u swapchain: %dx%d, %u images", i, sc.width, sc.height, imageCount);
    }

    glGenFramebuffers(1, &fbo_);
    eyeRenderers_[0].init();
    eyeRenderers_[1].init();

    return true;
}

void XrAppState::pollXrEvents(android_app *app) {
    XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
    while (instance_ != XR_NULL_HANDLE
           && xrPollEvent(instance_, &event) == XR_SUCCESS) {
        switch (event.type) {
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                auto *stateEvent = reinterpret_cast<XrEventDataSessionStateChanged *>(&event);
                sessionState_ = stateEvent->state;
                LOGI("Session state changed: %d", static_cast<int>(sessionState_));
                if (sessionState_ == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo beginInfo{XR_TYPE_SESSION_BEGIN_INFO};
                    beginInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                    xrBeginSession(session_, &beginInfo);
                    sessionRunning_ = true;
                } else if (sessionState_ == XR_SESSION_STATE_STOPPING) {
                    xrEndSession(session_);
                    sessionRunning_ = false;
                } else if (sessionState_ == XR_SESSION_STATE_EXITING
                           || sessionState_ == XR_SESSION_STATE_LOSS_PENDING) {
                    requestedExit_ = true;
                }
                break;
            }
            case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING:
                requestedExit_ = true;
                break;
            default:
                break;
        }
        event = {XR_TYPE_EVENT_DATA_BUFFER};
    }
    (void)app;
}

void XrAppState::renderEyeToSwapchain(uint32_t eyeIndex, XrSwapchainSubImage &outSubImage) {
    Swapchain &sc = swapchains_[eyeIndex];

    XrSwapchainImageAcquireInfo acquireInfo{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    uint32_t imageIndex = 0;
    xrCheck(xrAcquireSwapchainImage(sc.handle, &acquireInfo, &imageIndex), "xrAcquireSwapchainImage");

    XrSwapchainImageWaitInfo waitInfo{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    waitInfo.timeout = XR_INFINITE_DURATION;
    xrCheck(xrWaitSwapchainImage(sc.handle, &waitInfo), "xrWaitSwapchainImage");

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                            sc.images[imageIndex].image, 0);
    glViewport(0, 0, sc.width, sc.height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Same left/right selection PovFetcher exposes -- eyeIndex 0 is always left per
    // xrEnumerateViewConfigurationViews' STEREO ordering (left, then right).
    PovFetcher::Frame frame;
    bool hasFrame = (eyeIndex == 0) ? povFetcher_.latestLeft(frame) : povFetcher_.latestRight(frame);
    if (hasFrame) {
        eyeRenderers_[eyeIndex].updateTexture(frame.rgba.data(), frame.width, frame.height);
    }
    eyeRenderers_[eyeIndex].draw();

    XrSwapchainImageReleaseInfo releaseInfo{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    xrReleaseSwapchainImage(sc.handle, &releaseInfo);

    outSubImage.swapchain = sc.handle;
    outSubImage.imageRect.offset = {0, 0};
    outSubImage.imageRect.extent = {sc.width, sc.height};
    outSubImage.imageArrayIndex = 0;
}

void XrAppState::renderFrame() {
    XrFrameWaitInfo waitInfo{XR_TYPE_FRAME_WAIT_INFO};
    XrFrameState frameState{XR_TYPE_FRAME_STATE};
    if (!xrCheck(xrWaitFrame(session_, &waitInfo, &frameState), "xrWaitFrame")) return;

    XrFrameBeginInfo beginInfo{XR_TYPE_FRAME_BEGIN_INFO};
    xrCheck(xrBeginFrame(session_, &beginInfo), "xrBeginFrame");

    // XrCompositionLayerQuad (one per eye, via eyeVisibility), NOT XrCompositionLayerProjection.
    // A projection layer tells the compositor "this image IS a perspective-correct render for
    // this exact pose+fov" and it reprojects/lens-warps accordingly -- correct for real 3D
    // content, but this is a flat 2D video feed from the ROBOT's own camera (a different fov
    // entirely from the headset's). Feeding the headset's actual pose+fov for content that
    // wasn't rendered with that fov is what caused a live-confirmed bug: the source image
    // (verified correct -- pulled and inspected pov_left.png directly, a normal ~50deg
    // downward-tilted desk view) rendered in-headset as a distorted near-top-down view. A quad
    // layer sidesteps this entirely: it's just "place this flat rectangle at this fixed pose in
    // this reference space," so the compositor doesn't reinterpret the image's own perspective
    // at all -- the standard OpenXR pattern for flat video/image panels (as opposed to
    // real-time-rendered 3D scenes).
    std::vector<XrCompositionLayerBaseHeader *> layers;
    XrCompositionLayerQuad quadLayers[2]{};
    quadLayers[0].type = XR_TYPE_COMPOSITION_LAYER_QUAD;
    quadLayers[1].type = XR_TYPE_COMPOSITION_LAYER_QUAD;

    if (frameState.shouldRender) {
        // 1.5m in front, matching index.html's HUD-locked POV quad depth convention
        // (drawPovQuad's quadZ=-1.5). Size in meters preserves the source images' actual 16:9
        // aspect ratio (1280x720) -- 1.6m wide reads as a comfortably large "screen" at that
        // distance without dominating the whole view.
        constexpr float kDistanceM = 1.5f;
        constexpr float kWidthM = 1.6f;
        constexpr float kHeightM = kWidthM * (720.0f / 1280.0f);

        for (uint32_t i = 0; i < 2; ++i) {
            XrCompositionLayerQuad &quad = quadLayers[i];
            quad.space = localSpace_;
            quad.eyeVisibility = (i == 0) ? XR_EYE_VISIBILITY_LEFT : XR_EYE_VISIBILITY_RIGHT;
            quad.pose.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
            quad.pose.position = {0.0f, 0.0f, -kDistanceM};
            quad.size = {kWidthM, kHeightM};
            renderEyeToSwapchain(i, quad.subImage);
            layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader *>(&quad));
        }
    }

    XrFrameEndInfo endInfo{XR_TYPE_FRAME_END_INFO};
    endInfo.displayTime = frameState.predictedDisplayTime;
    endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    endInfo.layerCount = static_cast<uint32_t>(layers.size());
    endInfo.layers = layers.data();
    XrResult endResult = xrEndFrame(session_, &endInfo);

    static bool loggedFirstFrame = false;
    if (!loggedFirstFrame) {
        loggedFirstFrame = true;
        LOGI("First renderFrame() completed: shouldRender=%d xrEndFrame result=%d",
             frameState.shouldRender, static_cast<int>(endResult));
    }
}

void XrAppState::shutdown() {
    povFetcher_.stop();
    if (session_ != XR_NULL_HANDLE) xrDestroySession(session_);
    if (instance_ != XR_NULL_HANDLE) xrDestroyInstance(instance_);
    if (eglDisplay_ != EGL_NO_DISPLAY) {
        eglMakeCurrent(eglDisplay_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (eglContext_ != EGL_NO_CONTEXT) eglDestroyContext(eglDisplay_, eglContext_);
        if (eglSurface_ != EGL_NO_SURFACE) eglDestroySurface(eglDisplay_, eglSurface_);
        eglTerminate(eglDisplay_);
    }
}

void XrAppState::run(android_app *app) {
    app_ = app;
    app->userData = this;
    app->onAppCmd = onAppCmd;

    while (!requestedExit_ && !app->destroyRequested) {
        int events = 0;
        android_poll_source *source = nullptr;
        // Non-blocking poll while the session is running (so the render loop keeps ticking).
        // NOT -1 (indefinite block) otherwise: the OpenXR IDLE->READY transition (and other XR
        // events) arrive via xrPollEvent below, a completely separate queue from Android's
        // ALooper -- blocking here indefinitely starves pollXrEvents() of ever running again,
        // so the session gets stuck in IDLE forever (confirmed live: instance/session/swapchain
        // all created successfully, but session state never advanced past IDLE=1 until this was
        // changed to a short timeout instead of -1). 10ms keeps CPU usage reasonable while
        // still polling XR events promptly pre-session-start.
        int timeoutMs = sessionRunning_ ? 0 : 10;
        while (ALooper_pollOnce(timeoutMs, nullptr, &events, reinterpret_cast<void **>(&source)) >= 0) {
            if (source != nullptr) source->process(app, source);
            if (app->destroyRequested) break;
            timeoutMs = 0;  // drain any remaining queued events without blocking
        }

        pollXrEvents(app);

        if (sessionRunning_) {
            renderFrame();
        }
    }

    shutdown();
}
