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

    def log_request(self, code="-", size="-"):
        # index.html polls pov_left/right.jpg, wrist_cam_*.jpg and marker_uv.json back-to-back,
        # so the per-GET access log floods the console and buries the sim's [Quest][...] prints.
        # log_request, NOT log_message: log_error() routes through log_message, so overriding
        # that would also swallow 404s and 500s -- exactly the errors behind a blank headset.
        pass


if __name__ == "__main__":
    if not (CERT_FILE.exists() and KEY_FILE.exists()):
        CERT_DIR.mkdir(parents=True, exist_ok=True)
        print(f"Generating self-signed SSL certificates in {CERT_DIR}...")
        import subprocess
        subprocess.run([
            "openssl", "req", "-x509", "-newkey", "rsa:2048",
            "-keyout", str(KEY_FILE),
            "-out", str(CERT_FILE),
            "-days", "365", "-nodes",
            "-subj", "/CN=localhost"
        ], check=True)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)  # Change HTTP to HTTPS.
    ctx.load_cert_chain(  # Certs to make it secure.
        certfile=str(CERT_FILE),
        keyfile=str(KEY_FILE),
    )

    server = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    print(f"Serving at https://0.0.0.0:{PORT}")
    server.serve_forever()
