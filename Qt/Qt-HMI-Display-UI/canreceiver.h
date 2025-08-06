#ifndef CANRECEIVER_H
#define CANRECEIVER_H

#include <QObject>
#include <QSocketNotifier>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>


struct __attribute__((packed)) VehicleStatus {
    uint8_t engine_status;
    uint8_t light_status;
    uint8_t tire_pressure;
    uint8_t door_status;
    uint8_t seat_belt_status;
    uint8_t battery_level;
    uint8_t speed;
    uint8_t arrived_distance;
    uint8_t remain_distance;
    uint8_t drived_time;
    uint8_t transmission_gear;
    uint8_t reserved1;
    uint8_t reserved2;
    float gps_lat;
    float gps_lon;
};

struct controlData_t {
    uint8_t air_condition_temperature;
    uint8_t speed_limit;
    uint8_t light_touch_control;
};

struct DataTransfer_t {
    VehicleStatus status;
    controlData_t control_data;
};

class CanReceiver : public QObject {
    Q_OBJECT
    Q_PROPERTY(int speed READ speed NOTIFY dataUpdated)
    Q_PROPERTY(int engine_status READ engineStatus NOTIFY dataUpdated)
    Q_PROPERTY(int light_status READ lightStatus NOTIFY dataUpdated)
    Q_PROPERTY(int tire_pressure READ tirePressure NOTIFY dataUpdated)
    Q_PROPERTY(int door_status READ doorStatus NOTIFY dataUpdated)
    Q_PROPERTY(int seat_belt_status READ seatBeltStatus NOTIFY dataUpdated)
    Q_PROPERTY(int battery_level READ batteryLevel NOTIFY dataUpdated)
    Q_PROPERTY(int arrived_distance READ arrivedDistance NOTIFY dataUpdated)
    Q_PROPERTY(int remain_distance READ remainDistance NOTIFY dataUpdated)
    Q_PROPERTY(int drived_time READ drivedTime NOTIFY dataUpdated)
    Q_PROPERTY(int transmission_gear READ transmissionGear NOTIFY dataUpdated)
    Q_PROPERTY(float gps_lat READ gpsLat NOTIFY dataUpdated)
    Q_PROPERTY(float gps_lon READ gpsLon NOTIFY dataUpdated)
    Q_PROPERTY(int air_condition_temperature READ airConditionTemperature NOTIFY dataUpdated)
    Q_PROPERTY(int speed_limit READ speedLimit NOTIFY dataUpdated)

public:
    explicit CanReceiver(QObject *parent = nullptr);
    ~CanReceiver();
    int speed() const { return m_speed; }
    int engineStatus() const { return m_engineStatus; }
    int lightStatus() const { return m_lightStatus; }
    int tirePressure() const { return m_tirePressure; }
    int doorStatus() const { return m_doorStatus; }
    int seatBeltStatus() const { return m_seatBeltStatus; }
    int batteryLevel() const { return m_batteryLevel; }
    int arrivedDistance() const { return m_arrivedDistance; }
    int remainDistance() const { return m_remainDistance; }
    int drivedTime() const { return m_drivedTime; }
    int transmissionGear() const { return m_transmissionGear; }
    float gpsLat() const { return m_gpsLat; }
    float gpsLon() const { return m_gpsLon; }
    int airConditionTemperature() const { return m_airConditionTemperature; }
    int speedLimit() const { return m_speedLimit; }
    
public slots:
    void airConditionTemperatureAdd() { m_airConditionTemperature++; emit dataUpdated(); }
    void airConditionTemperatureSubtract() { m_airConditionTemperature--; emit dataUpdated(); }
    void speedLimitAdd() { m_speedLimit++; emit dataUpdated(); }
    void speedLimitSubtract() { m_speedLimit--; emit dataUpdated(); }
    void setLightStatus(int status) { m_lightStatus = status; emit dataUpdated(); }
    void UpdateData() 
    {
        controlData_t controlData;
        controlData.air_condition_temperature = m_airConditionTemperature;
        controlData.speed_limit = m_speedLimit;
        controlData.light_touch_control = m_lightStatus;
        sendto(m_sockfd, &controlData, sizeof(controlData_t), 0, (struct sockaddr *)&m_canHandlerProcessAddr, sizeof(m_canHandlerProcessAddr));
    }
signals:
    void dataUpdated();

private slots:
    void readPendingDatagram();

private:
    int m_sockfd;
    struct sockaddr_un m_canHandlerProcessAddr;
    QSocketNotifier *m_notifier;
    int m_speed;
    int m_engineStatus;
    int m_lightStatus;
    int m_tirePressure;
    int m_doorStatus;
    int m_seatBeltStatus;
    int m_batteryLevel;
    int m_arrivedDistance;
    int m_remainDistance;
    int m_drivedTime;
    int m_transmissionGear;
    float m_gpsLat;
    float m_gpsLon;
    int m_airConditionTemperature;
    int m_speedLimit; 
};

#endif // CANRECEIVER_H
