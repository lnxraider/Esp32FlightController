#include "Communication.h"
#include <Wire.h>

// Constructor to initialize GPS and SBUS with the given serial objects
Communication::Communication(HardwareSerial& gpsSerial, HardwareSerial& sbusSerial)
    : serialGPS(gpsSerial), sbus(&sbusSerial, SBUS_RX_PIN, SBUS_TX_PIN,  true)
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
    //while (serialGPS.available() > 0) {
    //    gps.encode(serialGPS.read());
    //}
    updateGPS();
}

void Communication::processGPSData() {
    if (gps.location.isUpdated()) {
        // Process GPS location data
        currentLatitude  = gps.location.lat();
        currentLongitude = gps.location.lng();
        Serial.print("Latitude: "); Serial.println(currentLatitude, 6);
        Serial.print("Longitude: "); Serial.println(currentLongitude, 6);
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

// Fetch the current GPS latitude
float Communication::getGPSLatitude() {
    return currentLatitude;
}

// Fetch the current GPS longitude
float Communication::getGPSLongitude() {
    return currentLongitude;
}

// Check if the SBUS signal has been lost (failsafe trigger)
bool Communication::isSignalLost() {
    bool _result = false;
    if (sbusData.failsafe || sbusData.lost_frame) {
        _result = true;
    }
    return _result;
}

void Communication::updateIMU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    float accelRoll = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
    float accelPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
    
    complementaryRoll = ALPHA * (complementaryRoll + g.gyro.x * dt) + (1 - ALPHA) * accelRoll;
    complementaryPitch = ALPHA * (complementaryPitch + g.gyro.y * dt) + (1 - ALPHA) * accelPitch;
    
    imuData.roll = complementaryRoll;
    imuData.pitch = complementaryPitch;
    imuData.yaw += g.gyro.z * dt;
}

void Communication::updateGPS() {
    while (serialGPS.available() > 0) {
        if (gps.encode(serialGPS.read())) {
            if (gps.location.isUpdated()) {
                currentLatitude = gps.location.lat();
                currentLongitude = gps.location.lng();
            }
        }
    }
}
