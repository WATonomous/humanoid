#!/usr/bin/env bash
# Configure and bring up a native SocketCAN interface (e.g. a candleLight / gs_usb adapter,
# USB id 1d50:606f). Only touches the network link; never sends a CAN frame.
#
# Usage: setup_socketcan.sh [iface=can0] [bitrate=1000000] [--listen-only]
set -euo pipefail

IFACE="can0"
BITRATE="1000000"
LISTEN_ONLY="off"

positional=()
for arg in "$@"; do
    case "$arg" in
        --listen-only) LISTEN_ONLY="on" ;;
        -h|--help)
            echo "Usage: $0 [iface=can0] [bitrate=1000000] [--listen-only]"
            exit 0
            ;;
        *) positional+=("$arg") ;;
    esac
done
[ "${#positional[@]}" -ge 1 ] && IFACE="${positional[0]}"
[ "${#positional[@]}" -ge 2 ] && BITRATE="${positional[1]}"

if ! [[ "$BITRATE" =~ ^[0-9]+$ ]]; then
    echo "Bitrate must be an integer in bps, got '$BITRATE'." >&2
    exit 1
fi

SUDO=""
if [ "${EUID:-$(id -u)}" -ne 0 ]; then
    if command -v sudo >/dev/null 2>&1; then
        SUDO="sudo"
    else
        echo "Needs root (or sudo) to configure $IFACE." >&2
        exit 1
    fi
fi

if ! command -v ip >/dev/null 2>&1; then
    echo "'ip' not found. Install iproute2." >&2
    exit 1
fi

if ! ip link show "$IFACE" >/dev/null 2>&1; then
    cat >&2 <<EOF
Interface '$IFACE' does not exist. Checks:
  lsusb | grep 1d50:606f      # is the gs_usb adapter enumerated?
  lsmod | grep gs_usb         # is the driver loaded?
  sudo modprobe gs_usb        # load it if not
  ip -details link show type can
EOF
    exit 1
fi

details="$(ip -details link show "$IFACE")"
if ! grep -q "link/can" <<<"$details"; then
    echo "'$IFACE' is not a CAN interface; refusing to touch it." >&2
    exit 1
fi

$SUDO ip link set "$IFACE" down

# vcan has no controller, so no bitrate/restart settings.
if grep -q "can state" <<<"$details"; then
    # No restart-ms: gs_usb rejects it ("doesn't support restart from Bus Off").
    can_opts=(bitrate "$BITRATE")
    [ "$LISTEN_ONLY" = "on" ] && can_opts+=(listen-only on)
    $SUDO ip link set "$IFACE" type can "${can_opts[@]}"
else
    echo "'$IFACE' is a virtual CAN link; skipping bitrate."
fi

$SUDO ip link set "$IFACE" txqueuelen 1000
$SUDO ip link set "$IFACE" up

echo
ip -details -statistics link show "$IFACE"
echo
state="$(ip -details link show "$IFACE" | grep -o 'can state [A-Z-]*' || true)"
if [ -n "$state" ]; then
    echo "$IFACE is up: $state (bitrate $BITRATE, listen-only $LISTEN_ONLY)"
    cat <<'EOF'
  ERROR-ACTIVE              healthy
  ERROR-PASSIVE / BUS-OFF   after a transmit: nothing is ACKing. The bus is unwired, has no
                            120 ohm termination, or no powered node is on it.
                            Re-run this script to recover.
EOF
else
    echo "$IFACE is up."
fi
