#!/bin/bash

while true; do
    # Cấu hình lại wifi và mạng
    pkill wpa_supplicant
    pkill dhclient
    ip link set wlu1u1 down
    ip link set wlu1u1 up
    sleep 1

    iw dev wlu1u1 connect "realme10"
    sleep 2
    udhcpc -i wlu1u1
    sleep 2
    route add default gw 10.216.60.131 wlu1u1
    echo "nameserver 8.8.8.8" > /etc/resolv.conf

    # Kiểm tra mạng
    ping -c 1 -W 2 8.8.8.8 > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "Internet OK!"
        break
    else
        echo "Chưa có mạng, thử lại sau 2 giây..."
        sleep 1
    fi
done

CAN_IFACE="can0"
BITRATE=500000

ip link set $CAN_IFACE down 2>/dev/null
ip link set $CAN_IFACE type can bitrate $BITRATE
ip link set $CAN_IFACE up

ip -details link show $CAN_IFACE

export QT_QPA_PLATFORM=linuxfb
export QT_QPA_EVDEV_TOUCHSCREEN_PARAMETERS="/dev/input/event1"

/root/Qt/Qt-HMI-Display-UI/Car_1 &
/root/WarningSound/WarningProcess &
/root/TelegramNotify/TelegramNotifyProcess &
/root/MainProcess/GatewayProcess &
/root/SpeechRecognitionProcess/speechRecognition &

wait
