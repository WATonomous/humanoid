#include "pov_fetcher.h"

#include <android/log.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstring>
#include <chrono>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define LOG_TAG "questopenxr.pov"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace {

// Blocking, single-request-per-connection HTTP/1.1 GET (mirrors what index.html's
// fetch(path, {cache:"no-store"}) does each poll tick) -- deliberately minimal, no keep-alive,
// no chunked-transfer support, since webxr_server.py's http.server always sends a plain
// Content-Length body for a static file GET.
bool httpGet(const std::string &host, int port, const char *path, std::vector<uint8_t> &body) {
    struct addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo *result = nullptr;
    char portStr[16];
    snprintf(portStr, sizeof(portStr), "%d", port);
    if (getaddrinfo(host.c_str(), portStr, &hints, &result) != 0 || result == nullptr) {
        LOGE("getaddrinfo failed for %s:%d", host.c_str(), port);
        return false;
    }

    int sock = -1;
    for (struct addrinfo *rp = result; rp != nullptr; rp = rp->ai_next) {
        sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sock < 0) continue;
        if (connect(sock, rp->ai_addr, rp->ai_addrlen) == 0) break;
        close(sock);
        sock = -1;
    }
    freeaddrinfo(result);
    if (sock < 0) {
        LOGE("connect failed for %s:%d", host.c_str(), port);
        return false;
    }

    // Without this, a connection left half-open by e.g. an `adb reverse` drop/reconnect (seen
    // live: happens whenever the headset goes idle/reconnects) hangs the blocking recv() loop
    // below forever -- and since this function runs sequentially in PovFetcher's single
    // background thread, one stuck connection silently stalls ALL future fetches (both eyes),
    // which read as "black screen, no errors, nothing" with zero log output to explain why.
    struct timeval timeout{};
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    std::string req = std::string("GET ") + path + " HTTP/1.1\r\n"
        + "Host: " + host + "\r\n"
        + "Connection: close\r\n"
        + "Cache-Control: no-cache\r\n\r\n";
    if (send(sock, req.data(), req.size(), 0) < 0) {
        LOGE("send failed");
        close(sock);
        return false;
    }

    // Read the whole response (headers + body) into one buffer -- POV frames are small
    // (a few hundred KB at most), so buffering the full response before parsing is fine.
    std::vector<uint8_t> raw;
    raw.reserve(1 << 20);
    uint8_t chunk[8192];
    ssize_t n;
    while ((n = recv(sock, chunk, sizeof(chunk), 0)) > 0) {
        raw.insert(raw.end(), chunk, chunk + n);
    }
    close(sock);

    if (raw.empty()) return false;

    // Split headers/body on the first blank line (\r\n\r\n).
    const char *headerEnd = nullptr;
    for (size_t i = 0; i + 3 < raw.size(); ++i) {
        if (raw[i] == '\r' && raw[i + 1] == '\n' && raw[i + 2] == '\r' && raw[i + 3] == '\n') {
            headerEnd = reinterpret_cast<const char *>(&raw[i]);
            size_t headerLen = i;
            std::string headers(reinterpret_cast<const char *>(raw.data()), headerLen);
            if (headers.find("200") == std::string::npos) {
                // Not a 200 OK (e.g. 404 before the sim has written its first POV frame yet).
                return false;
            }
            size_t bodyStart = i + 4;
            body.assign(raw.begin() + static_cast<long>(bodyStart), raw.end());
            return !body.empty();
        }
    }
    (void)headerEnd;
    return false;
}

}  // namespace

void PovFetcher::start(std::string host, int port) {
    host_ = std::move(host);
    port_ = port;
    running_ = true;
    thread_ = std::thread(&PovFetcher::run, this);
}

void PovFetcher::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

bool PovFetcher::fetchAndDecode(const char *path, Frame &out) {
    std::vector<uint8_t> body;
    if (!httpGet(host_, port_, path, body)) return false;

    int w = 0, h = 0, channels = 0;
    uint8_t *pixels = stbi_load_from_memory(body.data(), static_cast<int>(body.size()), &w, &h,
                                             &channels, 4 /* force RGBA */);
    if (pixels == nullptr) {
        LOGE("stbi_load_from_memory failed for %s (%zu bytes)", path, body.size());
        return false;
    }

    out.width = w;
    out.height = h;
    out.rgba.assign(pixels, pixels + (static_cast<size_t>(w) * h * 4));
    stbi_image_free(pixels);
    return true;
}

void PovFetcher::run() {
    LOGI("PovFetcher started, host=%s port=%d", host_.c_str(), port_);
    while (running_) {
        auto start = std::chrono::steady_clock::now();

        Frame left;
        if (fetchAndDecode("/pov_left.png", left)) {
            left.version = ++versionCounter_;
            std::lock_guard<std::mutex> lock(leftMutex_);
            leftFrame_ = std::move(left);
        }

        Frame right;
        if (fetchAndDecode("/pov_right.png", right)) {
            right.version = ++versionCounter_;
            std::lock_guard<std::mutex> lock(rightMutex_);
            rightFrame_ = std::move(right);
        }

        // Same 33ms poll interval as index.html's povFetchInterval, for consistency with the
        // already-tuned backend capture rate (_POV_CAPTURE_EVERY_N_STEPS in
        // run_quest_armv2_teleop.py).
        auto elapsed = std::chrono::steady_clock::now() - start;
        auto target = std::chrono::milliseconds(33);
        if (elapsed < target) {
            std::this_thread::sleep_for(target - elapsed);
        }
    }
    LOGI("PovFetcher stopped");
}

bool PovFetcher::latestLeft(Frame &out) {
    std::lock_guard<std::mutex> lock(leftMutex_);
    if (leftFrame_.width == 0) return false;
    out = leftFrame_;
    return true;
}

bool PovFetcher::latestRight(Frame &out) {
    std::lock_guard<std::mutex> lock(rightMutex_);
    if (rightFrame_.width == 0) return false;
    out = rightFrame_;
    return true;
}
