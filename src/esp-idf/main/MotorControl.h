#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

class MotorControl {
public:
    void arm();
    void disarm();
    bool isArmed() const { return armed; }
    void initialize();
    void controlMotors(float throttle, float roll, float pitch, float yaw);
    void updateArming();

private:
    void writeMotorPWM(int motorIndex, float value);
    const int motorPins[4] = {15, 2, 4, 16};  // Update with your actual motor pins
    const int minPWM = 1000;  // Min PWM signal (microseconds)
    const int maxPWM = 2000;  // Max PWM signal (microseconds)
    bool armed = false;

    unsigned long armStartTime = 0;
    const unsigned long ARM_DURATION = 3000; // 3 seconds
};

#endif  // MOTORCONTROL_H

