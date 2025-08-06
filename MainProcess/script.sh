#!/bin/bash

CAN_IFACE="can1"
BITRATE=500000

if [[ $EUID -ne 0 ]]; then
  echo "Chay voi quyen root (sudo)."
  exit 1
fi

config-pin P9_24 can
config-pin P9_26 can

ip link set $CAN_IFACE down 2>/dev/null
ip link set $CAN_IFACE type can bitrate $BITRATE
ip link set $CAN_IFACE up

ip -details link show $CAN_IFACE

ip route add default via 192.168.7.1 dev usb0
echo "nameserver 8.8.8.8" > /etc/resolv.conf

export QT_QPA_PLATFORM=linuxfb
export QT_QPA_EVDEV_TOUCHSCREEN_PARAMETERS="/dev/input/event1"

/root/Qt/Qt-HMI-Display-UI/Car_1 &