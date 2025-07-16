#ifndef CANRECEIVER_H
#define CANRECEIVER_H

#include <QObject>
#include <QSocketNotifier>

// struct VehicleStatus {
//     uint8_t engine_status;
//     uint8_t light_status;
//     uint8_t tire_pressure;
//     uint8_t door_status;
//     uint8_t seat_belt_status;
//     uint8_t battery_level;
//     uint8_t speed;
//     uint8_t arrived_distance;
//     uint8_t remain_distance;
//     uint8_t avg_speed;
//     uint8_t engine_temperature;
//     uint8_t transmission_gear;
//     uint8_t speed_limit;
//     float gps_lat;
//     float gps_lon;
// };

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
    uint8_t avg_speed;
    uint8_t engine_temperature;
    uint8_t transmission_gear;
    uint8_t speed_limit;
    float gps_lat;
    float gps_lon;
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
    Q_PROPERTY(int avg_speed READ avgSpeed NOTIFY dataUpdated)
    Q_PROPERTY(int engine_temperature READ engineTemperature NOTIFY dataUpdated)
    Q_PROPERTY(int transmission_gear READ transmissionGear NOTIFY dataUpdated)
    Q_PROPERTY(int speed_limit READ speedLimit NOTIFY dataUpdated)
    Q_PROPERTY(float gps_lat READ gpsLat NOTIFY dataUpdated)
    Q_PROPERTY(float gps_lon READ gpsLon NOTIFY dataUpdated)

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
    int avgSpeed() const { return m_avgSpeed; }
    int engineTemperature() const { return m_engineTemperature; }
    int transmissionGear() const { return m_transmissionGear; }
    int speedLimit() const { return m_speedLimit; }
    float gpsLat() const { return m_gpsLat; }
    float gpsLon() const { return m_gpsLon; }

signals:
    void dataUpdated();

private slots:
    void readPendingDatagram();

private:
    int m_sockfd;
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
    int m_avgSpeed;
    int m_engineTemperature;
    int m_transmissionGear;
    int m_speedLimit;
    float m_gpsLat;
    float m_gpsLon;
};

#endif // CANRECEIVER_H
