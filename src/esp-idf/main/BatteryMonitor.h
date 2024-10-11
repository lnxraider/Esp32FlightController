#ifndef BATTERYMONITOR_H
#define BATTERYMONITOR_H

class BatteryMonitor {
public:
    void initialize();
    void monitorBattery();

private:
    float readBatteryVoltage();
    const float lowBatteryThreshold = 3.5;  // Voltage threshold
};

#endif  // BATTERYMONITOR_H

