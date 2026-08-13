#!/usr/bin/env python3

import http.server
import ssl
import os
import threading
from pathlib import Path

PORT = 8443
# Plain-HTTP mirror of the same static dir, for the native OpenXR app (quest_openxr_app) --
# unlike the browser's WebXR path, a native app has no secure-context requirement forcing HTTPS,
# and skipping TLS entirely avoids needing a TLS/cert stack in the C++ HTTP client.
PLAIN_PORT = 8080

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

STATIC_DIR = PACKAGE_DIR / "static"
CERT_DIR = Path(os.environ.get("TELEOP_CERT_DIR", PACKAGE_DIR / "certs"))

CERT_FILE = CERT_DIR / "cert.pem"
KEY_FILE = CERT_DIR / "key.pem"


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        # Serves files in the static directory over HTTPS.
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)


def _serve_plain_http() -> None:
    plain_server = http.server.ThreadingHTTPServer(("0.0.0.0", PLAIN_PORT), Handler)
    print(f"Serving at http://0.0.0.0:{PLAIN_PORT} (plain HTTP, for quest_openxr_app)")
    plain_server.serve_forever()


if __name__ == "__main__":
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)  # Change HTTP to HTTPS.
    ctx.load_cert_chain(  # Certs to make it secure.
        certfile=str(CERT_FILE),
        keyfile=str(KEY_FILE),
    )

    threading.Thread(target=_serve_plain_http, daemon=True).start()

    server = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    print(f"Serving at https://0.0.0.0:{PORT}")
    server.serve_forever()
