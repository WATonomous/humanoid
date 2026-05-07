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
<<<<<<< HEAD
        # Serves files in the static directory over HTTPS.
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)


if __name__ == "__main__":
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)  # Change HTTP to HTTPS.
    ctx.load_cert_chain(  # Certs to make it secure.
=======
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs) # Servers file in teh static directory over the https


if __name__ == "__main__":
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER) # Change http to https
    ctx.load_cert_chain( # Certs to make it secure
>>>>>>> cbcbea1d (new changes)
        certfile=str(CERT_FILE),
        keyfile=str(KEY_FILE),
    )

<<<<<<< HEAD
    server = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    print(f"Serving at https://0.0.0.0:{PORT}")
    server.serve_forever()
=======
    server = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler) # Creates http server
    server.socket = ctx.wrap_socket(server.socket, server_side=True) # uses TLS to change http -> https

    print(f"Serving at https://0.0.0.0:{PORT}")
    server.serve_forever()
>>>>>>> cbcbea1d (new changes)
