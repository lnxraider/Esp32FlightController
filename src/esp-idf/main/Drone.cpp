#include "Drone.h"

Drone::Drone()
    : flightControl(&communication)  // Pass Communication instance to FlightControl
{
    // Empty constructor
}

bool Drone::initialize() {
    if (!communication.initialize()) {
        Serial.println("Communication initialization failed!");
        return false;
    }
    motorControl.initialize();
    batteryMonitor.initialize();
    return true;
}

void Drone::updateFlightControl() {
    flightControl.mapReceiverInput();
    flightControl.tunePID();
    motorControl.controlMotors(
        flightControl.getThrottle(),
        flightControl.getRoll(),
        flightControl.getPitch(),
        flightControl.getYaw()
    );
    batteryMonitor.monitorBattery();
}

void Drone::updateCommunication() {
    communication.updateSensors();
    communication.processGPSData();
}

