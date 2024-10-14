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
