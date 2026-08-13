cmake_minimum_required(VERSION 3.22.1)
project(questopenxr)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party)

# --- OpenXR loader (vendored Khronos SDK source, built from source) ---
set(BUILD_LOADER ON CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_CONFORMANCE_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_API_LAYERS OFF CACHE BOOL "" FORCE)
add_subdirectory(${THIRD_PARTY_DIR}/OpenXR-SDK ${CMAKE_CURRENT_BINARY_DIR}/openxr-sdk-build)

# --- android_native_app_glue (NDK-provided source, not a prebuilt lib) ---
add_library(native_app_glue STATIC
    ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c
)
target_include_directories(native_app_glue PUBLIC
    ${ANDROID_NDK}/sources/android/native_app_glue
)

add_library(questopenxr SHARED
    main.cpp
    xr_app.cpp
    gl_renderer.cpp
    pov_fetcher.cpp
)

target_include_directories(questopenxr PRIVATE
    ${THIRD_PARTY_DIR}/stb
)

# Required so android_native_app_glue's android_main is not stripped by the linker as unused
# (NativeActivity's Java-side glue calls it by symbol name via dlsym, not a normal call site).
target_link_options(questopenxr PRIVATE "-u" "ANativeActivity_onCreate")

target_link_libraries(questopenxr
    PRIVATE
    native_app_glue
    openxr_loader
    android
    log
    EGL
    GLESv3
    m
)
