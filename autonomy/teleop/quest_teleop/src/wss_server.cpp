#include "quest_teleop/wss_server.hpp"

#include <iostream>

#include <boost/beast/core.hpp>
<<<<<<< HEAD
#include <boost/beast/http.hpp>
=======
>>>>>>> cbcbea1d (new changes)
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>

namespace beast = boost::beast;
<<<<<<< HEAD
namespace http = beast::http;
=======
>>>>>>> cbcbea1d (new changes)
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
<<<<<<< HEAD
    ssl::stream<tcp::socket> tls_stream(std::move(socket), ssl_context_);
    tls_stream.handshake(ssl::stream_base::server);

    // Read the first request to distinguish plain HTTP from WebSocket upgrade.
    // Plain HTTP arrives when the Quest browser navigates to https://localhost:9090
    // to accept the self-signed cert — serve a redirect so the cert trust flow
    // completes cleanly before the WebSocket connection is attempted.
    beast::flat_buffer buffer;
    http::request<http::string_body> req;
    http::read(tls_stream, buffer, req);

    if (!websocket::is_upgrade(req)) {
      http::response<http::string_body> res{http::status::ok, req.version()};
      res.set(http::field::content_type, "text/html");
      res.body() = "<html><body><h1>Certificate trusted.</h1>"
                   "<p>Return to <a href='https://localhost:8443'>https://localhost:8443</a> and "
                   "press Start.</p>"
                   "</body></html>";
      res.prepare_payload();
      http::write(tls_stream, res);
      return;
    }

    websocket::stream<ssl::stream<tcp::socket>> ws(std::move(tls_stream));
    ws.accept(req);
=======
    websocket::stream<ssl::stream<tcp::socket>> ws(std::move(socket), ssl_context_);

    ws.next_layer().handshake(ssl::stream_base::server);
    ws.accept();
>>>>>>> cbcbea1d (new changes)

    std::cout << "Quest browser connected" << std::endl;

    while (running_) {
<<<<<<< HEAD
      beast::flat_buffer msg_buffer;
      ws.read(msg_buffer);

      std::string text = beast::buffers_to_string(msg_buffer.data());
=======
      beast::flat_buffer buffer;
      ws.read(buffer);

      std::string text = beast::buffers_to_string(buffer.data());
>>>>>>> cbcbea1d (new changes)

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