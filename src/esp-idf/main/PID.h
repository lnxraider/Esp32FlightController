#ifndef PID_H
#define PID_H

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

