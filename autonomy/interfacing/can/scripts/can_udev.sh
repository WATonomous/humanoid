#!/usr/bin/env bash
set -e

UDEV_RULE_FILE="/etc/udev/rules.d/99-canable.rules"
UDEV_VENDOR_ID="16d0"
UDEV_PRODUCT_ID="117e"
UDEV_SERIAL_DEFAULT="208B38B43136"
UDEV_SYMLINK_DEFAULT="canable"

SUDO=""
if [ "${EUID:-$(id -u)}" -ne 0 ]; then
    if command -v sudo >/dev/null 2>&1; then
        SUDO="sudo"
    fi
fi

usage() {
    echo "Usage: $0 install [serial] [symlink]"
    echo "       $0 remove"
}

install_rule() {
    local serial="$1"
    local symlink="$2"

    if [ -z "$serial" ]; then
        serial="$UDEV_SERIAL_DEFAULT"
    fi
    if [ -z "$symlink" ]; then
        symlink="$UDEV_SYMLINK_DEFAULT"
    fi

    if [ -z "$SUDO" ] && [ "${EUID:-$(id -u)}" -ne 0 ]; then
        echo "This command requires root privileges (or sudo)."
        exit 1
    fi

    echo "Setting up CANable udev rule..."

    cat <<EOF | $SUDO tee "$UDEV_RULE_FILE" > /dev/null
SUBSYSTEM=="tty", ATTRS{idVendor}=="$UDEV_VENDOR_ID", ATTRS{idProduct}=="$UDEV_PRODUCT_ID", ATTRS{serial}=="$serial", SYMLINK+="$symlink"
EOF

    echo "Reloading udev rules..."
    $SUDO udevadm control --reload-rules
    $SUDO udevadm trigger

    sleep 1

    if [ -e "/dev/$symlink" ]; then
        echo "✔ /dev/$symlink created successfully"
    else
        echo "⚠ /dev/$symlink not found yet — try unplugging and replugging the CANable"
    fi
}

remove_rule() {
    if [ -z "$SUDO" ] && [ "${EUID:-$(id -u)}" -ne 0 ]; then
        echo "This command requires root privileges (or sudo)."
        exit 1
    fi

    if [ -e "$UDEV_RULE_FILE" ]; then
        echo "Removing udev rule: $UDEV_RULE_FILE"
        $SUDO rm -f "$UDEV_RULE_FILE"
        echo "Reloading udev rules..."
        $SUDO udevadm control --reload-rules
        $SUDO udevadm trigger
    else
        echo "udev rule not present: $UDEV_RULE_FILE"
    fi
}

CMD="$1"
case "$CMD" in
    install)
        install_rule "$2" "$3"
        ;;
    remove)
        remove_rule
        ;;
    -h|--help|help|"" )
        usage
        exit 1
        ;;
    *)
        usage
        exit 1
        ;;
esac
