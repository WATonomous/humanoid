#include "quest_teleop/ws_server.hpp"

#include <iostream>

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;

WsServer::WsServer(std::uint16_t port, MessageCallback on_message)
    : port_(port), on_message_(on_message), acceptor_(io_context_), running_(false) {}

WsServer::~WsServer() {
  stop();
}

void WsServer::start() {
  if (running_) {
    return;
  }

  running_ = true;
  server_thread_ = std::thread(&WsServer::run, this);
}

void WsServer::stop() {
  running_ = false;

  boost::system::error_code ec;
  acceptor_.close(ec);
  io_context_.stop();

  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}

void WsServer::run() {
  try {
    tcp::endpoint endpoint(tcp::v4(), port_);

    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(boost::asio::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();

    std::cout << "WS (plain) server listening on port " << port_ << std::endl;

    while (running_) {
      tcp::socket socket(io_context_);
      acceptor_.accept(socket);

      std::thread(&WsServer::handle_session, this, std::move(socket)).detach();
    }
  } catch (const std::exception& e) {
    if (running_) {
      std::cerr << "WS server error: " << e.what() << std::endl;
    }
  }
}

void WsServer::handle_session(tcp::socket socket) {
  try {
    websocket::stream<tcp::socket> ws(std::move(socket));
    ws.accept();

    std::cout << "Quest OpenXR app connected (plain WS)" << std::endl;

    while (running_) {
      beast::flat_buffer msg_buffer;
      ws.read(msg_buffer);

      std::string text = beast::buffers_to_string(msg_buffer.data());

      if (on_message_) {
        on_message_(text);
      }

      ws.text(true);
      ws.write(boost::asio::buffer(std::string("ok")));
    }
  } catch (const std::exception& e) {
    std::cout << "WS session ended: " << e.what() << std::endl;
  }
}
