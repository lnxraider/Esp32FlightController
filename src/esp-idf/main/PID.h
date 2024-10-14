#ifndef PID_H
#define PID_H


const float MAX_OUTPUT = 1000.0;  // Assuming motor speed range is 0-1000
const float MAX_INTEGRAL = 0.2 * MAX_OUTPUT;
const float dt = 0.01;  // 1/100th of a second

class PID {
public:
    PID(float kp, float ki, float kd);
    void setTunings(float kp, float ki, float kd);
    float calculate(float setpoint, float input);

private:
    float kp;
    float ki;
    float kd;
    float previousError;
    float integral;
};

#endif  // PID_H

