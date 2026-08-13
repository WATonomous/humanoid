#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Fetches pov_left.png / pov_right.png from webxr_server.py's plain-HTTP mirror (port 8080,
// see webxr_server.py's PLAIN_PORT) on a background thread, decodes with stb_image, and hands
// off decoded RGBA8 frames to the render thread via a mutex-protected "latest frame" slot per
// eye -- same double-buffer-by-copy pattern index.html uses (pendingLeftBitmap/pendingRightBitmap),
// just re-implemented without a browser. No TLS: a native app has no secure-context restriction
// forcing HTTPS the way WebXR in a browser does, so this deliberately talks to the plain-HTTP
// port instead of pulling in a TLS stack for a same-network capture-and-display loop.
class PovFetcher {
public:
    struct Frame {
        std::vector<uint8_t> rgba;
        int width = 0;
        int height = 0;
        uint64_t version = 0;  // bumped on every new decoded frame, so the renderer can skip a re-upload if unchanged
    };

    // host is the machine running webxr_server.py -- reachable from the Quest exactly like the
    // browser path (same Wi-Fi network, or `adb reverse tcp:8080 tcp:8080` if tethered/USB).
    void start(std::string host, int port = 8080);
    void stop();

    // Returns a copy of the latest decoded frame for the given eye ("pov_left.png"/"pov_right.png").
    // Returns false if no frame has been decoded yet.
    bool latestLeft(Frame &out);
    bool latestRight(Frame &out);

private:
    void run();
    bool fetchAndDecode(const char *path, Frame &out);

    std::string host_;
    int port_ = 8080;
    std::atomic<bool> running_{false};
    std::thread thread_;

    std::mutex leftMutex_;
    Frame leftFrame_;
    std::mutex rightMutex_;
    Frame rightFrame_;
    uint64_t versionCounter_ = 0;
};
