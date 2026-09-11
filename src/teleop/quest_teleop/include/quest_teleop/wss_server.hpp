#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>

class WssServer {
public:
  using MessageCallback = std::function<void(const std::string&)>;

  WssServer(std::uint16_t port, const std::string& cert_dir, MessageCallback on_message);

  ~WssServer();

  void start();
  void stop();

private:
  using tcp = boost::asio::ip::tcp;

  void run();
  void handle_session(tcp::socket socket);
  void load_certificates(const std::string& cert_dir);

  std::uint16_t port_;
  std::string cert_dir_;
  MessageCallback on_message_;

  boost::asio::io_context io_context_;
  boost::asio::ssl::context ssl_context_;
  tcp::acceptor acceptor_;

  std::thread server_thread_;
  std::atomic_bool running_;
};