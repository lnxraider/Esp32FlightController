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

float PID::calculate(float setpoint, float input) {
    float error = setpoint - input;
    integral += error;
    float derivative = error - previousError;
    previousError = error;

    return kp * error + ki * integral + kd * derivative;
}

