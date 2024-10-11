#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

#include "PID.h"
#include "Communication.h"

class FlightControl {
public:
    FlightControl(Communication* comm);
    void mapReceiverInput();
    void tunePID();

    float getThrottle();
    float getRoll();
    float getPitch();
    float getYaw();

private:
    PID pidThrottle;
    PID pidRoll;
    PID pidPitch;
    PID pidYaw;

    Communication* communication;

    float throttle;
    float roll;
    float pitch;
    float yaw;

    // Desired setpoints (e.g., from receiver inputs)
    float desiredThrottle;
    float desiredRoll;
    float desiredPitch;
    float desiredYaw;
};

#endif  // FLIGHTCONTROL_H

