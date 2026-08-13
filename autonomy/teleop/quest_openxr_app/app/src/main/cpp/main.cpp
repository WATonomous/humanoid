#include <android_native_app_glue.h>

#include "xr_app.h"

// NativeActivity entry point (called by android_native_app_glue's ANativeActivity_onCreate,
// which the linker keeps only because of CMakeLists.txt's -u ANativeActivity_onCreate flag).
void android_main(android_app *app) {
    XrAppState state;
    state.run(app);
}
