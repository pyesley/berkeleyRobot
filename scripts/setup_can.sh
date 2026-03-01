#!/bin/bash
# Setup CAN interface for FYSETC UCAN adapter (gs_usb based)
# Run with sudo

set -e

echo "Setting up CAN interface..."

# Load CAN modules
modprobe can
modprobe can_raw

# Load gs_usb driver for FYSETC UCAN
modprobe gs_usb

# Wait a moment for the interface to appear
sleep 1

# Check if can0 exists
if ! ip link show can0 &>/dev/null; then
    echo "ERROR: can0 interface not found"
    echo "Check that FYSETC UCAN is plugged in:"
    echo "  lsusb | grep 1d50:606f"
    exit 1
fi

# Bring down if already up
ip link set can0 down 2>/dev/null || true

# Configure CAN interface
# 1000000 baud to match Recoil motor controller (STM32 FDCAN @ 1Mbps)
ip link set can0 type can bitrate 1000000 restart-ms 100

# Increase TX queue length (default 10 is too small)
ip link set can0 txqueuelen 128

# Bring up the interface
ip link set can0 up

echo "CAN interface can0 is now up at 1Mbps"
echo ""
echo "Verify with: ip -details link show can0"
echo "Monitor with: candump can0"
