#!/usr/bin/env python3

import http.server
import ssl
import os
from pathlib import Path
from urllib.parse import urlsplit

PORT = 8443

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

SHARED_STATIC = Path("/workspace/isaaclab/logs/static")
if not SHARED_STATIC.exists():
    SHARED_STATIC = Path(os.path.expanduser("~/IsaacLab/logs/static"))
SHARED_STATIC.mkdir(parents=True, exist_ok=True)

if os.environ.get("TELEOP_STATIC_DIR"):
    STATIC_DIR = Path(os.environ["TELEOP_STATIC_DIR"])
else:
    STATIC_DIR = SHARED_STATIC
CERT_DIR = Path(os.environ.get("TELEOP_CERT_DIR", PACKAGE_DIR / "certs"))

CERT_FILE = CERT_DIR / "cert.pem"
KEY_FILE = CERT_DIR / "key.pem"


class Handler(http.server.SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"  # Reuse TLS connections, eliminating TCP/SSL handshake latency

    def __init__(self, *args, **kwargs):
        # Camera JPEGs live in the shared Isaac Lab static directory, but the
        # application page must always come from this package so a git pull
        # immediately activates WebXR/control-path changes.
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def translate_path(self, path):
        if urlsplit(path).path in ("/", "/index.html"):
            return str(PACKAGE_DIR / "static" / "index.html")
        return super().translate_path(path)

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

    http.server.ThreadingHTTPServer.allow_reuse_address = True
    server = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    print(f"Serving at https://0.0.0.0:{PORT}")
    server.serve_forever()
