#include "PID.h"

PID::PID(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), previousError(0), integral(0)
{
}

void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

/*
float PID::calculate(float setpoint, float input) {
    float error = setpoint - input;
    integral += error;
    float derivative = error - previousError;
    previousError = error;

    return kp * error + ki * integral + kd * derivative;
}
*/

float PID::calculate(float setpoint, float input) {
    float error = setpoint - input;
    integral += ki * error;

    // Anti-windup
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float derivative = -kd * (input - previousInput) / dt;  // Derivative on measurement
    previousInput = input;

    float output = kp * error + integral + derivative;

    // Output clamping
    return constrain(output, -MAX_OUTPUT, MAX_OUTPUT);
}

void PID::updateDynamicTunings(const FlightConditions& conditions) {
    // Example adjustment based on battery voltage
    float voltageRatio = conditions.batteryVoltage / NOMINAL_BATTERY_VOLTAGE;
    kp *= voltageRatio;
    ki *= voltageRatio;
    kd *= voltageRatio;

    // Adjust based on payload weight
    float weightFactor = 1 + (conditions.payloadWeight / DRONE_BASE_WEIGHT);
    kp *= weightFactor;
    ki /= weightFactor;  // Reduce integral gain to prevent overshoot with heavier payloads

    // Adjust based on altitude
    float altitudeFactor = 1 + (conditions.altitude / BASE_ALTITUDE);
    kp *= altitudeFactor;
    kd *= altitudeFactor;

    // Adjust for wind conditions
    if (conditions.windSpeed > WIND_THRESHOLD) {
        kp *= 1.2;  // Increase proportional gain for better disturbance rejection
        kd *= 1.5;  // Increase derivative gain for faster response
    }

    // Adjust based on flight speed
    if (conditions.flightSpeed > HIGH_SPEED_THRESHOLD) {
        kp *= 0.8;  // Reduce proportional gain to prevent oscillations at high speeds
        kd *= 1.2;  // Increase derivative gain for better damping
    }

    // Ensure gains stay within safe limits
    kp = constrain(kp, MIN_KP, MAX_KP);
    ki = constrain(ki, MIN_KI, MAX_KI);
    kd = constrain(kd, MIN_KD, MAX_KD);
}


