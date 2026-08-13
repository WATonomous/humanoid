#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include <boost/asio.hpp>

// Plain (non-TLS) mirror of WssServer, on a separate port -- added specifically for
// quest_openxr_app (the native OpenXR client), which needs a WebSocket connection to send hand
// tracking data back for actual teleop control (the browser/WebXR client already works fine
// over WssServer's TLS connection; a native app has no browser secure-context requirement
// forcing WSS the way index.html's WebXR APIs do). Implementing a full TLS stack from scratch
// in the Android app would be a much larger, separate undertaking than just adding this plain
// listener here -- same reasoning as webxr_server.py's PLAIN_PORT mirror for POV image fetches.
class WsServer {
public:
  using MessageCallback = std::function<void(const std::string&)>;

  WsServer(std::uint16_t port, MessageCallback on_message);

  ~WsServer();

  void start();
  void stop();

private:
  using tcp = boost::asio::ip::tcp;

  void run();
  void handle_session(tcp::socket socket);

  std::uint16_t port_;
  MessageCallback on_message_;

  boost::asio::io_context io_context_;
  tcp::acceptor acceptor_;

  std::thread server_thread_;
  std::atomic_bool running_;
};
