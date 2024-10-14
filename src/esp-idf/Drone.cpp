#include "Drone.h"

Drone::Drone()
    : flightControl(&communication)  // Pass Communication instance to FlightControl
{
    // Empty constructor
}

bool Drone::initialize() {
    if (!communication.initialize()) {
        logError(DroneError::COMMUNICATION_FAILURE);
        return false;
    }
    if (!motorControl.initialize()) {
        logError(DroneError::MOTOR_FAILURE);
        return false;
    }
    batteryMonitor.initialize();
    return true;
}

void Drone::updateFlightControl() {
    flightControl.update();
    motorControl.controlMotors(
        flightControl.getThrottle(),
        flightControl.getRoll(),
        flightControl.getPitch(),
        flightControl.getYaw()
    );
    if (!batteryMonitor.monitorBattery()) {
        setFlightMode(FlightControl::RETURN_TO_HOME);
        logError(DroneError::BATTERY_CRITICAL);
    };
}

void Drone::updateCommunication() {
    communication.updateSensors();
    communication.processGPSData();
}

void Drone::logError(DroneError error) {
    lastError = error;
    // Log to SD card or transmit error code
    Serial.print("Error: ");
    Serial.println(static_cast<int>(error));
}

void Drone::writeLogError(const char* message) {
    File logFile = SD.open("/error_log.txt", FILE_APPEND);
    if (logFile) {
        logFile.println(message);
        logFile.close();
    }
}
