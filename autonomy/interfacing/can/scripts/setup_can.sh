#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

DEVICE_PATH="$1"
INTERFACE_NAME="$2"
BITRATE_CODE="$3" # e.g., -s6 for 500k, -s8 for 1M

if [ -z "$DEVICE_PATH" ] || [ -z "$INTERFACE_NAME" ] || [ -z "$BITRATE_CODE" ]; then
    echo "Usage: $0 <device_path> <interface_name> <bitrate_code>"
    echo "Example: $0 /dev/ttyACM0 can0 -s6"
    exit 1
fi

echo "Attempting to set up CAN interface $INTERFACE_NAME on $DEVICE_PATH with bitrate code $BITRATE_CODE..."

# Check if slcand is already running for this device/interface to avoid errors
if pgrep -f "slcand.*$DEVICE_PATH.*$INTERFACE_NAME"; then
    echo "slcand appears to be already running for $DEVICE_PATH and $INTERFACE_NAME. Assuming interface is (or will be) up."
else
    echo "Starting slcand: slcand -o -c $BITRATE_CODE $DEVICE_PATH $INTERFACE_NAME"
    slcand -o -c "$BITRATE_CODE" "$DEVICE_PATH" "$INTERFACE_NAME"
    # Add a small delay to allow slcand to establish the interface
    sleep 1
fi

echo "Bringing up interface: ifconfig $INTERFACE_NAME up"
ifconfig "$INTERFACE_NAME" up

echo "Setting txqueuelen: ifconfig $INTERFACE_NAME txqueuelen 1000"
ifconfig "$INTERFACE_NAME" txqueuelen 1000

echo "CAN interface $INTERFACE_NAME setup complete."
exit 0