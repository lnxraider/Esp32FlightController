#include "FlightControl.h"
#include <Arduino.h>

FlightControl::FlightControl(Communication* comm)
    : communication(comm),
      pidThrottle(1.0, 0.0, 0.0),
      pidRoll(1.0, 0.0, 0.0),
      pidPitch(1.0, 0.0, 0.0),
      pidYaw(1.0, 0.0, 0.0)
{
    // Initialize PIDs if necessary
}

void FlightControl::mapReceiverInput() {
    bfs::SbusData sbusData = communication->getSbusData();

    // Map SBUS channels to control inputs
    desiredThrottle = map(sbusData.ch(THR), 172, 1811, 0, 1);  // Normalize to 0-1
    desiredRoll     = map(sbusData.ch(AIL), 172, 1811, -1, 1); // Normalize to -1 to 1
    desiredPitch    = map(sbusData.ch(ELE), 172, 1811, -1, 1); // Normalize to -1 to 1
    desiredYaw      = map(sbusData.ch(RUD), 172, 1811, -1, 1); // Normalize to -1 to 1
}

void FlightControl::tunePID() {
    // Dynamic PID tuning via SBUS channels (if desired)
    bfs::SbusData sbusData = communication->getSbusData();

    float kp = map(sbusData.ch(4), 172, 1811, 0.0, 5.0);  // Channel 5 for Kp
    float ki = map(sbusData.ch(5), 172, 1811, 0.0, 1.0);  // Channel 6 for Ki
    float kd = map(sbusData.ch(6), 172, 1811, 0.0, 1.0);  // Channel 7 for Kd

    pidRoll.setTunings(kp, ki, kd);
    pidPitch.setTunings(kp, ki, kd);
    pidYaw.setTunings(kp, ki, kd);
}

float FlightControl::getThrottle() {
    // For throttle, we might bypass PID for direct control
    return desiredThrottle;
}

float FlightControl::getRoll() {
    // Use PID to calculate the necessary roll correction
    float currentRoll = communication->getIMUData().roll;
    roll = pidRoll.calculate(desiredRoll, currentRoll);
    return roll;
}

float FlightControl::getPitch() {
    float currentPitch = communication->getIMUData().pitch;
    pitch = pidPitch.calculate(desiredPitch, currentPitch);
    return pitch;
}

float FlightControl::getYaw() {
    float currentYaw = communication->getIMUData().yaw;
    yaw = pidYaw.calculate(desiredYaw, currentYaw);
    return yaw;
}

