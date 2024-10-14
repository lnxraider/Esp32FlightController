#ifndef BATTERYMONITOR_H
#define BATTERYMONITOR_H

class BatteryMonitor {
public:
    void initialize();
    bool monitorBattery();

    // battery thresholds
    enum class BatteryState { 
	    NORMAL, 
	    WARNING, 
	    CRITICAL, 
	    EMERGENCY 
    };

private:
    float readBatteryVoltage();
    const float lowBatteryThreshold = 3.5;  // Voltage threshold

    BatteryState checkBatteryState();
    const int VOLTAGE_SAMPLES = 10;
    float voltageReadings[VOLTAGE_SAMPLES];
    int readIndex = 0;
    float voltageTotal = 0;

};
#endif  // BATTERYMONITOR_H

