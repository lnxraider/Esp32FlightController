#ifndef PID_H
#define PID_H

// PID Safe Limits
// Proportional Gain (Kp)
const float MIN_KP = 0.1;
const float MAX_KP = 1.0;

// Integral Gain (Ki)
const float MIN_KI = 0.001;
const float MAX_KI = 0.100;

// Derivative Gain (Kd)
const float MIN_KD = 0.01;
const float MAX_KD = 0.50;

const float MAX_OUTPUT = 1000.0;  // Assuming motor speed range is 0-1000
const float MAX_INTEGRAL = 0.2 * MAX_OUTPUT;
const float dt = 0.01;  // 1/100th of a second

// Dynamic Tuning
struct FlightConditions {
    float batteryVoltage;
    float payloadWeight;
    float altitude;
    float windSpeed;
    float flightSpeed;
};

class PID {
public:
    PID(float kp, float ki, float kd);
    void setTunings(float kp, float ki, float kd);
    float calculate(float setpoint, float input);
    void updateDynamicTunings(const FlightConditions& conditions);

private:
    float kp;
    float ki;
    float kd;
    float previousError;
    float integral;
};

#endif  // PID_H

