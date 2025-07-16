#!/bin/bash

# Thiết lập giao diện CAN1 cho BeagleBone Black
CAN_IFACE="can1"
BITRATE=500000

echo " Cau hinh $CAN_IFACE voi toc do $BITRATE..."

# Kiểm tra quyền root
if [[ $EUID -ne 0 ]]; then
  echo "Chay voi quyen root (sudo)."
  exit 1
fi

sudo config-pin P9_24 can
sudo config-pin P9_26 can

# Gỡ cấu hình nếu đang bật
ip link set $CAN_IFACE down 2>/dev/null

# Thiết lập lại với tốc độ mong muốn
ip link set $CAN_IFACE type can bitrate $BITRATE

# Bật giao diện
ip link set $CAN_IFACE up

# Hiển thị trạng thái
echo "Cau hinh $CAN_IFACE hoan tat:"
ip -details link show $CAN_IFACE

export QT_QPA_PLATFORM=linuxfb