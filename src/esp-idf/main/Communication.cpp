#include "Communication.h"
#include <Wire.h>

Communication::Communication()
    : sbus(&Serial2, &sbusData),  // Use Serial2 for SBUS
      serialGPS(1)                // Use Serial1 for GPS
{
    // Empty constructor
}

bool Communication::initialize() {
    // Initialize BMP280
    if (!bmp280.begin(0x76)) {
        Serial.println("Failed to initialize BMP280!");
        return false;
    }

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050!");
        return false;
    }

    // Initialize GPS
    serialGPS.begin(9600, SERIAL_8N1, 16, 17);  // GPIO16 RX, GPIO17 TX

    // Initialize SBUS
    if (!sbus.Begin()) {
        Serial.println("Failed to initialize SBUS!");
        return false;
    }

    return true;
}

void Communication::updateSensors() {
    // Update SBUS data
    if (sbus.Read()) {
        sbusData = sbus.data();
    }

    // Update IMU data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    imuData.roll = a.acceleration.x;  // Simplified for example
    imuData.pitch = a.acceleration.y;
    imuData.yaw = g.gyro.z;

    // Update altitude from BMP280
    altitude = bmp280.readAltitude(1013.25);  // Adjust sea level pressure as needed

    // Update GPS data
    while (serialGPS.available() > 0) {
        gps.encode(serialGPS.read());
    }
}

void Communication::processGPSData() {
    if (gps.location.isUpdated()) {
        // Process GPS location data
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        Serial.print("Latitude: "); Serial.println(latitude, 6);
        Serial.print("Longitude: "); Serial.println(longitude, 6);
    }
}

bfs::SbusData Communication::getSbusData() {
    return sbusData;
}

IMUData Communication::getIMUData() {
    return imuData;
}

float Communication::getAltitude() {
    return altitude;
}

