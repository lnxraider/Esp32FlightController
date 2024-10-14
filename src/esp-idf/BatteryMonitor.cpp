#include "BatteryMonitor.h"
#include <Arduino.h>

void BatteryMonitor::initialize() {
    // Initialize ADC for battery voltage reading
    analogReadResolution(12);  // 12-bit resolution
}

bool BatteryMonitor::monitorBattery() {
    bool _result = true;
    float voltage = readBatteryVoltage();
    if (voltage < lowBatteryThreshold) {
        Serial.println("Low battery! Initiating failsafe.");
        // Implement failsafe action (e.g., return to home)
	_result = false;
    }
    return _result;
}

/*
float BatteryMonitor::readBatteryVoltage() {
    int adcValue = analogRead(34);  // Assuming battery voltage connected to GPIO34
    float voltage = (adcValue / 4095.0) * 3.3 * (voltageDividerRatio);  // Adjust for voltage divider
    return voltage;
}
*/

float BatteryMonitor::readBatteryVoltage() {
    voltageTotal -= voltageReadings[readIndex];
    voltageReadings[readIndex] = (analogRead(34) / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
    voltageTotal += voltageReadings[readIndex];
    readIndex = (readIndex + 1) % VOLTAGE_SAMPLES;

    return voltageTotal / VOLTAGE_SAMPLES;
}

BatteryState BatteryMonitor::checkBatteryState() {
    float voltage = readBatteryVoltage();
    if (voltage < EMERGENCY_THRESHOLD) return BatteryState::EMERGENCY;
    if (voltage < CRITICAL_THRESHOLD) return BatteryState::CRITICAL;
    if (voltage < WARNING_THRESHOLD) return BatteryState::WARNING;
    return BatteryState::NORMAL;
}
