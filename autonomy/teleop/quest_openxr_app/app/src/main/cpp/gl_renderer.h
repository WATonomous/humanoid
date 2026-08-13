#pragma once

#include <GLES3/gl3.h>
#include <cstdint>

// Minimal textured-quad renderer -- draws whatever RGBA8 buffer is uploaded to it, filling the
// current viewport/framebuffer. Used once per eye per frame in xr_app.cpp: bind that eye's
// OpenXR swapchain image as the GL framebuffer, then draw() the corresponding POV texture into
// it. Equivalent to index.html's drawPovQuad, just native GL instead of WebGL.
class GlRenderer {
public:
    bool init();
    void updateTexture(const uint8_t *rgba, int width, int height);
    void draw();

private:
    GLuint program_ = 0;
    GLuint vbo_ = 0;
    GLuint texture_ = 0;
    GLint uTextureLoc_ = -1;
    bool textureAllocated_ = false;
    int texWidth_ = 0;
    int texHeight_ = 0;
};
