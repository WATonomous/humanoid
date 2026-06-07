#include "quest_teleop/wss_server.hpp"

#include <iostream>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace ssl = boost::asio::ssl;

WssServer::WssServer(std::uint16_t port, const std::string& cert_dir, MessageCallback on_message)
    : port_(port), cert_dir_(cert_dir), on_message_(on_message),
      ssl_context_(ssl::context::tlsv12_server), acceptor_(io_context_), running_(false) {
  load_certificates(cert_dir_);
}

WssServer::~WssServer() {
  stop();
}

void WssServer::start() {
  if (running_) {
    return;
  }

  running_ = true;
  server_thread_ = std::thread(&WssServer::run, this);
}

void WssServer::stop() {
  running_ = false;

  boost::system::error_code ec;
  acceptor_.close(ec);
  io_context_.stop();

  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}

void WssServer::load_certificates(const std::string& cert_dir) {
  ssl_context_.set_options(ssl::context::default_workarounds | ssl::context::no_sslv2 |
                           ssl::context::single_dh_use);

  ssl_context_.use_certificate_chain_file(cert_dir + "/cert.pem");
  ssl_context_.use_private_key_file(cert_dir + "/key.pem", ssl::context::pem);
}

void WssServer::run() {
  try {
    tcp::endpoint endpoint(tcp::v4(), port_);

    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(boost::asio::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();

    std::cout << "WSS server listening on port " << port_ << std::endl;

    while (running_) {
      tcp::socket socket(io_context_);
      acceptor_.accept(socket);

      std::thread(&WssServer::handle_session, this, std::move(socket)).detach();
    }
  } catch (const std::exception& e) {
    if (running_) {
      std::cerr << "WSS server error: " << e.what() << std::endl;
    }
  }
}

void WssServer::handle_session(tcp::socket socket) {
  try {
    websocket::stream<ssl::stream<tcp::socket>> ws(std::move(socket), ssl_context_);

    ws.next_layer().handshake(ssl::stream_base::server);
    ws.accept();

    std::cout << "Quest browser connected" << std::endl;

    while (running_) {
      beast::flat_buffer buffer;
      ws.read(buffer);

      std::string text = beast::buffers_to_string(buffer.data());

      if (on_message_) {
        on_message_(text);
      }

      ws.text(true);
      ws.write(boost::asio::buffer(std::string("ok")));
    }
  } catch (const std::exception& e) {
    std::cout << "WSS session ended: " << e.what() << std::endl;
  }
}