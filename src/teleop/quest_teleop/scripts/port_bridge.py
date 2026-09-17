#!/usr/bin/env python3
"""Universal TCP Port Bridge for Rootless Docker on Slurm.

Binds to 0.0.0.0:8443 and 0.0.0.0:9090 on the host (trpro-slurm1) and pipes
all TCP traffic directly into the running container via `docker exec`.
Works in all rootless network namespaces without needing port mappings.
"""

import socket
import subprocess
import threading
import sys

CONTAINER_NAME = "isaac-lab-ros2"

def pump(src, dst):
    try:
        while True:
            data = src.recv(65536) if hasattr(src, "recv") else src.read(65536)
            if not data:
                break
            if hasattr(dst, "sendall"):
                dst.sendall(data)
            else:
                dst.write(data)
                dst.flush()
    except Exception:
        pass
    finally:
        try:
            src.close()
        except Exception:
            pass
        try:
            dst.close()
        except Exception:
            pass


def forward_port(host_port: int, container_port: int):
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", host_port))
        server.listen(50)
        print(f"[PortBridge] Listening on host 0.0.0.0:{host_port} -> container:{container_port}", flush=True)
    except OSError as e:
        print(f"[PortBridge] Port {host_port} on host already in use ({e}) -- skipping host bind.", flush=True)
        return

    # In-container forwarder snippet (uses container's bundled python3)
    container_cmd = (
        f"import socket, sys, threading; "
        f"s = socket.create_connection(('127.0.0.1', {container_port})); "
        f"def r():\n"
        f"    while True:\n"
        f"        b = s.recv(65536)\n"
        f"        if not b: break\n"
        f"        sys.stdout.buffer.write(b)\n"
        f"        sys.stdout.buffer.flush()\n"
        f"threading.Thread(target=r, daemon=True).start()\n"
        f"while True:\n"
        f"    b = sys.stdin.buffer.read(65536)\n"
        f"    if not b: break\n"
        f"    s.sendall(b)\n"
    )

    while True:
        client_sock, addr = server.accept()
        print(f"[PortBridge] Connection from {addr[0]}:{addr[1]} on port {host_port} -> forwarded to container:{container_port}", flush=True)
        proc = subprocess.Popen(
            ["docker", "exec", "-i", CONTAINER_NAME, "python3", "-c", container_cmd],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        threading.Thread(target=pump, args=(client_sock, proc.stdin), daemon=True).start()
        threading.Thread(target=pump, args=(proc.stdout, client_sock), daemon=True).start()


if __name__ == "__main__":
    import signal
    signal.signal(signal.SIGINT, lambda *_: (print("\n[PortBridge] Shutting down cleanly..."), sys.exit(0)))
    print(f"[PortBridge] Starting bridge for container '{CONTAINER_NAME}'...", flush=True)
    t = threading.Thread(target=forward_port, args=(8443, 8443), daemon=True)
    t.start()
    forward_port(9090, 9090)
    t.join()

