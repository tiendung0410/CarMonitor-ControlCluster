#include "canreceiver.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <QDebug>

#define SOCKET_PATH "/tmp/gui_socket"

CanReceiver::CanReceiver(QObject *parent)
    : QObject(parent), m_speed(0), m_engineStatus(0), m_lightStatus(0),
      m_tirePressure(0), m_doorStatus(0), m_seatBeltStatus(0), m_batteryLevel(0),
      m_arrivedDistance(0), m_remainDistance(0), m_avgSpeed(0),
      m_engineTemperature(0), m_transmissionGear(0), m_speedLimit(0),
      m_gpsLat(0.0), m_gpsLon(0.0)
{
    m_sockfd = socket(AF_UNIX, SOCK_DGRAM, 0);

    sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

    unlink(SOCKET_PATH); // tránh lỗi bind
    if (bind(m_sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        qFatal("Failed to bind socket");
    }

    m_notifier = new QSocketNotifier(m_sockfd, QSocketNotifier::Read, this);
    connect(m_notifier, &QSocketNotifier::activated, this, &CanReceiver::readPendingDatagram);
}


CanReceiver::~CanReceiver() {
    if (m_sockfd >= 0) close(m_sockfd);
    unlink(SOCKET_PATH);
}

void CanReceiver::readPendingDatagram() {
    VehicleStatus status;
    ssize_t n = recv(m_sockfd, &status, sizeof(status), 0);
    if (n == sizeof(status)) {
        m_engineStatus = status.engine_status;
        m_lightStatus = status.light_status;
        m_tirePressure = status.tire_pressure;
        m_doorStatus = status.door_status;
        m_seatBeltStatus = status.seat_belt_status;
        m_batteryLevel = status.battery_level;
        m_speed = status.speed;
        m_arrivedDistance = status.arrived_distance;
        m_remainDistance = status.remain_distance;
        m_avgSpeed = status.avg_speed;
        m_engineTemperature = status.engine_temperature;
        m_transmissionGear = status.transmission_gear;
        m_speedLimit = status.speed_limit;
        m_gpsLat = status.gps_lat;
        m_gpsLon = status.gps_lon;
        emit dataUpdated();
    }
}

