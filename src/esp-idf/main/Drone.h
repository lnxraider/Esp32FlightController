#ifndef DRONE_H
#define DRONE_H

#include "MotorControl.h"
#include "FlightControl.h"
#include "Communication.h"
#include "BatteryMonitor.h"

class Drone {
public:
    Drone();
    bool initialize();
    void updateFlightControl();
    void updateCommunication();

private:
    MotorControl motorControl;
    FlightControl flightControl;
    Communication communication;
    BatteryMonitor batteryMonitor;
};

#endif  // DRONE_H

