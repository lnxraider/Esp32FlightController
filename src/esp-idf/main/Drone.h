#ifndef DRONE_H
#define DRONE_H

#include "MotorControl.h"
#include "FlightControl.h"
#include "Communication.h"
#include "BatteryMonitor.h"
#include <optional>
#include <SD.h>

class Drone {
public:
    Drone();
    bool initialize();
    void updateFlightControl();
    void updateCommunication();

    enum class DroneError {
        COMMUNICATION_FAILURE,
        MOTOR_FAILURE,
        GPS_FAILURE,
        IMU_FAILURE,
        BATTERY_CRITICAL
    };

    void logError(DroneError error);
    void writeLogError(DroneError error);
    std::optional<DroneError> lastError;

private:
    MotorControl motorControl;
    FlightControl flightControl;
    Communication communication;
    BatteryMonitor batteryMonitor;
};

#endif  // DRONE_H

