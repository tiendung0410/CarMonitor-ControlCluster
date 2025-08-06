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
      m_arrivedDistance(0), m_remainDistance(0), m_drivedTime(0),
      m_transmissionGear(0),
      m_gpsLat(0.0), m_gpsLon(0.0), m_airConditionTemperature(25),m_speedLimit(80)
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

    int canProcessAddrLen = sizeof(m_canHandlerProcessAddr);
    DataTransfer_t dataTransfer;
    ssize_t n = recvfrom(m_sockfd, &dataTransfer, sizeof(DataTransfer_t), 0,(struct sockaddr *)&m_canHandlerProcessAddr,(socklen_t *)&canProcessAddrLen);
    if (n == sizeof(DataTransfer_t)) {
        m_engineStatus = dataTransfer.status.engine_status;
        m_lightStatus = dataTransfer.status.light_status;
        m_tirePressure = dataTransfer.status.tire_pressure;
        m_doorStatus = dataTransfer.status.door_status;
        m_seatBeltStatus = dataTransfer.status.seat_belt_status;
        m_batteryLevel = dataTransfer.status.battery_level;
        m_speed = dataTransfer.status.speed;
        m_arrivedDistance = dataTransfer.status.arrived_distance;
        m_remainDistance = dataTransfer.status.remain_distance;
        m_drivedTime = dataTransfer.status.drived_time;
        m_transmissionGear = dataTransfer.status.transmission_gear;
        m_gpsLat = dataTransfer.status.gps_lat;
        m_gpsLon = dataTransfer.status.gps_lon;
        m_airConditionTemperature = dataTransfer.control_data.air_condition_temperature;
        m_speedLimit = dataTransfer.control_data.speed_limit;
        emit dataUpdated();
    }
    
    
}

