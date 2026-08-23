#!/usr/bin/env python3

import http.server
import ssl
import os
from pathlib import Path

PORT = 8443

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

    def log_message(self, format, *args):
        # index.html polls pov_left/right.png, wrist_cam.png, contact_force_hud.png, and
        # marker_uv.json back-to-back with no fixed interval -- the default access log (one line
        # per GET) floods the console at dozens of lines/sec and buries the sim's own
        # [Quest][...] diagnostic prints. Silenced; flip to a plain `print` here if the raw
        # request log is ever needed again.
        pass


if __name__ == "__main__":
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)  # Change HTTP to HTTPS.
    ctx.load_cert_chain(  # Certs to make it secure.
        certfile=str(CERT_FILE),
        keyfile=str(KEY_FILE),
    )

    server = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    print(f"Serving at https://0.0.0.0:{PORT}")
    server.serve_forever()
