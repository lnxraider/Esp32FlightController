#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include "sbus.h"

#define GPS_RX_PIN 18
#define GPS_TX_PIN 19
#define SBUS_RX_PIN 16
#define SBUS_TX_PIN 17

// Define receiver channels
#define THR 0         // Throttle
#define AIL 1         // Aileron/Roll
#define ELE 2         // Elevator/Pitch
#define RUD 3         // Rudder/Yaw

// Struct to hold IMU data
struct IMUData {
    float roll;
    float pitch;
    float yaw;
};

class Communication {
public:
    Communication();
    bool initialize();
    void updateSensors();
    void processGPSData();

    bfs::SbusData getSbusData();
    IMUData getIMUData();
    float getAltitude();

private:
    Adafruit_BMP280 bmp280;
    Adafruit_MPU6050 mpu;
    TinyGPSPlus gps;
    HardwareSerial serialGPS;
    bfs::SbusRx sbus;
    bfs::SbusData sbusData;

    IMUData imuData;
    float altitude;
};

#endif  // COMMUNICATION_H

