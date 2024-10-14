#include "MotorControl.h"
#include <Arduino.h>

void MotorControl::initialize() {
    // Initialize motor control pins and PWM channels
    for (int i = 0; i < 4; i++) {
        ledcSetup(i, 50, 16);  // 50Hz frequency, 16-bit resolution
        ledcAttachPin(motorPins[i], i);
    }
}

void MotorControl::controlMotors(float throttle, float roll, float pitch, float yaw) {

    if (!armed) return;

    // Motor mixing for quadcopter in X configuration
    float motorSpeeds[4];
    motorSpeeds[0] = throttle + pitch + roll - yaw;  // Front Right
    motorSpeeds[1] = throttle + pitch - roll + yaw;  // Front Left
    motorSpeeds[2] = throttle - pitch + roll + yaw;  // Rear Right
    motorSpeeds[3] = throttle - pitch - roll - yaw;  // Rear Left

    // Write PWM values to motors
    for (int i = 0; i < 4; i++) {
        writeMotorPWM(i, motorSpeeds[i]);
    }
}

void MotorControl::writeMotorPWM(int motorIndex, float value) {
    // Ensure value is within allowed range
    value = constrain(value, 0.0, 1.0);  // Assuming throttle values between 0 and 1
    int pwmValue = map(value * 1000, 0, 1000, minPWM, maxPWM);  // Scale to PWM range
    ledcWrite(motorIndex, pwmValue);
}

void MotorControl::arm() {
    if (!armed) {
        // Perform arming sequence
        // e.g., set all motors to minimum throttle for a few seconds
        for (int i = 0; i < 4; i++) {
            writeMotorPWM(i, 0);
        }
        delay(3000); // Wait for 3 seconds
        armed = true;
    }
}

void MotorControl::disarm() {
    armed = false;
    for (int i = 0; i < 4; i++) {
        writeMotorPWM(i, 0);
    }
}

void updateArming() {
    if (armStartTime == 0) {
        armStartTime = millis();
        for (int i = 0; i < 4; i++) {
            writeMotorPWM(i, 0);
        }
    }

    if (millis() - armStartTime >= ARM_DURATION) {
        armed = true;
        armStartTime = 0;
    }
}

bool checkMotorHealth() {
    for (int i = 0; i < 4; i++) {
        if (getMotorRPM(i) < MIN_HEALTHY_RPM) {
            return false;
        }
    }
    return true;
}
