# quest_openxr_app

Native OpenXR (C++/NativeActivity, no Java/Kotlin) Quest app -- an alternative to the
browser/WebXR client (`autonomy/teleop/quest_teleop/static/index.html`) for displaying the
robot's stereo POV feed in-headset.

## Scope of this first pass

**Display only.** This renders `pov_left.png`/`pov_right.png` (the same files
`run_quest_armv2_teleop.py` already produces for the browser client) into both eyes via a real
OpenXR session/swapchain. It does **not** yet:

- Send hand/wrist tracking data back to `quest_teleop_node` (no `XR_EXT_hand_tracking`, no
  WebSocket client) -- so it can't currently be used to actually teleop the arms, only to look
  around at what the sim is rendering.
- Draw the wrist-target marker or hand-joint dots that `index.html` draws.

Wiring up hand tracking + a WebSocket client back to `quest_teleop_node` (port 9090) is the
natural next step if this display path checks out, but is real additional scope (OpenXR hand
tracking extension setup, plus a TLS WebSocket client in C++, which is a nontrivial addition on
top of the plain-HTTP GET client this uses for POV frames) -- not attempted here yet.

## Why this exists

Built in response to a live request to try a native OpenXR client instead of the WebXR-in-browser
approach, after investigating (and fixing, separately) the actual headset-fps bottleneck for the
existing browser client -- which turned out to be server-side (Isaac Sim render cost + a
recorder memory/RAM issue), not the browser/WebXR display path itself. This native app doesn't
address that bottleneck (same slow source frames either way); it exists because it was explicitly
requested, not as a performance fix.

## Building

```bash
export ANDROID_HOME=/home/rwahib/android-sdk
export PATH="/home/rwahib/gradle-8.7/bin:$PATH"
cd autonomy/teleop/quest_openxr_app
gradle assembleDebug
```

APK lands at `app/build/outputs/apk/debug/app-debug.apk`.

## Installing / running

```bash
adb install -r app/build/outputs/apk/debug/app-debug.apk
# webxr_server.py must be running (same as the browser workflow) with its new plain-HTTP mirror
# on port 8080 (see PLAIN_PORT in webxr_server.py) -- adb reverse it same as the existing
# tcp:8443/tcp:9090 pattern:
adb reverse tcp:8080 tcp:8080
adb shell am start -n com.wato.questopenxr/android.app.NativeActivity
adb logcat -s questopenxr.xr questopenxr.gl questopenxr.pov
```

`kPovHost` in `xr_app.cpp` defaults to `127.0.0.1` (works via the `adb reverse` above, matching
the project's existing USB-tethered workflow). Change it to the dev machine's LAN IP if testing
over Wi-Fi instead.

## Project layout

- `app/src/main/cpp/xr_app.{h,cpp}` -- OpenXR instance/session/swapchain lifecycle + per-frame
  render loop (the actual OpenXR boilerplate).
- `app/src/main/cpp/gl_renderer.{h,cpp}` -- minimal textured-quad GLES3 renderer, one instance
  per eye.
- `app/src/main/cpp/pov_fetcher.{h,cpp}` -- background thread, raw-socket plain-HTTP GET (no
  TLS -- see comment in pov_fetcher.h for why) + stb_image PNG decode, polling at the same 33ms
  cadence as `index.html`.
- `third_party/OpenXR-SDK` -- vendored Khronos OpenXR-SDK (loader source, built from source via
  CMake `add_subdirectory`).
- `third_party/stb/stb_image.h` -- vendored single-header PNG decoder.

## Status: verified working live

Built, installed, and confirmed live on a real Quest headset -- both eyes render the correct
stereo POV feed, matching the source `pov_left.png`/`pov_right.png` images.

Two real bugs were found and fixed getting there (both worth knowing about if this breaks again
after a change):

1. **Session stuck at `XR_SESSION_STATE_IDLE` forever.** The main loop blocked indefinitely
   (`ALooper_pollOnce(-1, ...)`) whenever the session wasn't yet running, on the assumption that
   nothing needs polling until an Android-side event wakes it up. But the IDLE->READY transition
   arrives via `xrPollEvent`, a completely separate queue from Android's `ALooper` -- blocking
   indefinitely starves it of ever running again. Fixed by polling on a short timeout (10ms)
   instead of blocking indefinitely when not yet running (see `run()` in `xr_app.cpp`).
2. **Correct source image rendered as a distorted near-top-down view in-headset.** Was using
   `XrCompositionLayerProjection` (2 per-eye views, fed the headset's *actual* current pose+fov)
   for what is actually flat 2D video from the *robot's own camera* -- a different fov entirely.
   The compositor's perspective-correct reprojection math, applied to content it wasn't actually
   valid for, warped the image. Switched to `XrCompositionLayerQuad` (one per eye via
   `eyeVisibility`) -- a flat panel at a fixed pose in the reference space, which sidesteps
   pose/fov reprojection semantics entirely. This is the standard OpenXR pattern for flat
   video/image content, not `XrCompositionLayerProjection` (reserved for genuine
   real-time-rendered 3D scenes).

Also needed, to get past Quest's system-level launch gating (not an app bug, just required
manifest declarations): `com.oculus.permission.HAND_TRACKING` permission,
`com.oculus.handtracking.frequency`/`.version` metadata, and a `oculus.software.handtracking`
`<uses-feature>` -- without these Quest assumes any VR app needs physical controllers and blocks
launch with a system dialog if none are paired, even though this app doesn't use hand tracking
data yet.

Still not done: hand tracking / sending control data back to `quest_teleop_node` (see "Scope of
this first pass" above) -- so this is a verified-working *viewer*, not yet a working teleop
client.
