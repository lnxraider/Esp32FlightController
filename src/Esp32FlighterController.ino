/***********************************************************************
* ChatGPT generated Flight Controller for the ESP32 Dev Module
*
*
***********************************************************************/

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// --- Global MPU6050 instance ---
Adafruit_MPU6050 mpu;

// --- GPS instance ---
TinyGPSPlus gps;
HardwareSerial serialGPS(1);  // GPS on Serial1

// --- Mutex for motor control and GPS data ---
SemaphoreHandle_t motorMutex;
SemaphoreHandle_t gpsMutex;

// --- Global variables for PID control ---
float kp_roll = 0.5, ki_roll = 0.05, kd_roll = 0.1;
float kp_pitch = 0.5, ki_pitch = 0.05, kd_pitch = 0.1;
float kp_yaw = 0.5, ki_yaw = 0.05, kd_yaw = 0.1;

float rollError = 0, pitchError = 0, yawError = 0;
float prevRollError = 0, prevPitchError = 0, prevYawError = 0;
float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;
float rollDerivative = 0, pitchDerivative = 0, yawDerivative = 0;
float prevAltitudeError = 0, altitudeIntegral = 0;

// --- Shared variables for control inputs ---
int throttleInput = 1000;
int pitchInput = 0, rollInput = 0, yawInput = 0;

// --- GPS data variables ---
double currentLatitude = 0.0;
double currentLongitude = 0.0;
double currentAltitude = 0.0;
double homeLatitude = 0.0;
double homeLongitude = 0.0;

// --- Battery and threshold ---
float batteryVoltage = 0.0;
float lowBatteryThreshold = 3.5;

// --- Constants ---
#define MOTOR1_PIN 5
#define MOTOR2_PIN 6
#define MOTOR3_PIN 7
#define MOTOR4_PIN 8
#define BATTERY_PIN 36  // ADC pin for battery

void setup() {
  Serial.begin(115200);
  serialGPS.begin(9600, SERIAL_8N1, 16, 17);  // Initialize GPS UART

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  Serial.println("MPU6050 Found!");

  // Configure MPU6050 ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 initialized and configured");

  // Initialize mutexes for motor control and GPS
  motorMutex = xSemaphoreCreateMutex();
  gpsMutex = xSemaphoreCreateMutex();

  // Create tasks
  xTaskCreate(flightControlTask, "Flight Control", 4096, NULL, 1, NULL);
  xTaskCreate(communicationTask, "Communication", 4096, NULL, 1, NULL);
}

void loop() {
  // Monitor battery voltage in the main loop
  monitorBattery();
}

// ----------------------- FLIGHT CONTROL TASK -----------------------
void flightControlTask(void *pvParameters) {
  while (true) {
    sensors_event_t accelEvent, gyroEvent, tempEvent;

    // Read MPU6050 sensor data
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    // Convert gyroscope readings to pitch, roll, yaw in degrees/s
    float pitch = gyroEvent.gyro.x * 57.2958;  
    float roll = gyroEvent.gyro.y * 57.2958;
    float yaw = gyroEvent.gyro.z * 57.2958;

    // --- Calculate PID corrections ---
    float rollCorrection = calculatePID(rollInput, roll, prevRollError, rollIntegral, kp_roll, ki_roll, kd_roll);
    float pitchCorrection = calculatePID(pitchInput, pitch, prevPitchError, pitchIntegral, kp_pitch, ki_pitch, kd_pitch);
    float yawCorrection = calculatePID(yawInput, yaw, prevYawError, yawIntegral, kp_yaw, ki_yaw, kd_yaw);

    // --- Safe motor control using mutex ---
    if (xSemaphoreTake(motorMutex, portMAX_DELAY)) {
      setMotorSpeeds(throttleInput, pitchCorrection, rollCorrection, yawCorrection);
      xSemaphoreGive(motorMutex);
    }

    // Run control loop at 50Hz
    delay(20);
  }
}

void setMotorSpeeds(int throttle, float pitchCorrection, float rollCorrection, float yawCorrection) {
  // Calculate motor speeds for a quadcopter
  int motor1Speed = throttle + pitchCorrection + rollCorrection + yawCorrection;
  int motor2Speed = throttle + pitchCorrection - rollCorrection - yawCorrection;
  int motor3Speed = throttle - pitchCorrection + rollCorrection - yawCorrection;
  int motor4Speed = throttle - pitchCorrection - rollCorrection + yawCorrection;

  // Constrain motor speeds to the range 1000-2000 (for PWM control)
  motor1Speed = constrain(motor1Speed, 1000, 2000);
  motor2Speed = constrain(motor2Speed, 1000, 2000);
  motor3Speed = constrain(motor3Speed, 1000, 2000);
  motor4Speed = constrain(motor4Speed, 1000, 2000);

  // Send motor speeds to motor control pins
  analogWrite(MOTOR1_PIN, motor1Speed);
  analogWrite(MOTOR2_PIN, motor2Speed);
  analogWrite(MOTOR3_PIN, motor3Speed);
  analogWrite(MOTOR4_PIN, motor4Speed);
}

// ----------------------- COMMUNICATION TASK -----------------------
void communicationTask(void *pvParameters) {
  while (true) {
    // --- GPS Data Handling ---
    while (serialGPS.available() > 0) {
      char c = serialGPS.read();
      gps.encode(c);  // Feed the GPS parser

      if (gps.location.isUpdated()) {
        // Safe access to GPS data using mutex
        if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
          currentLatitude = gps.location.lat();
          currentLongitude = gps.location.lng();
          currentAltitude = gps.altitude.meters();
          xSemaphoreGive(gpsMutex);
        }
      }
    }

    // --- UART Command Handling ---
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      handleCommand(command);
    }

    // Run communication task at 10Hz
    delay(100);
  }
}

void handleCommand(const String &command) {
  if (command.startsWith("throttle")) {
    int newThrottle = command.substring(9).toInt();
    throttleInput = constrain(newThrottle, 1000, 2000);  // Safely update throttle
  }
  if (command.startsWith("home")) {
    homeLatitude = currentLatitude;
    homeLongitude = currentLongitude;
    Serial.println("Home position set.");
  }
  // Additional commands can be handled here
}

// ------------------- HELPER FUNCTIONS -------------------
float calculatePID(float targetValue, float currentValue, float &prevError, float &integral, float kp, float ki, float kd) {
  float error = targetValue - currentValue;
  integral += error;
  float derivative = error - prevError;
  prevError = error;
  return (kp * error) + (ki * integral) + (kd * derivative);
}

float calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000; // Earth's radius in meters
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

float calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) -
            sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float bearing = atan2(y, x);
  return fmod((degrees(bearing) + 360), 360);  // Normalize to 0-360
}

float readBatteryVoltage() {
  int rawValue = analogRead(BATTERY_PIN);
  return (rawValue / 4095.0) * 3.3 * (11);  // Assuming a voltage divider of 11:1
}

void monitorBattery() {
  batteryVoltage = readBatteryVoltage();
  if (batteryVoltage < lowBatteryThreshold * 3) {  // For 3-cell LiPo (3.5V per cell)
    Serial.println("Low battery! Returning to home.");
    returnToHome();
  }
}

void returnToHome() {
  automatedFlight(homeLatitude, homeLongitude, currentAltitude);  // Fly to home
}

void automatedFlight(double targetLat, double targetLon, double targetAltitude) {
  while (true) {
    if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
      double distanceToTarget = calculateDistance(currentLatitude, currentLongitude, targetLat, targetLon);
      xSemaphoreGive(gpsMutex);

      if (distanceToTarget < 2.0) {
        Serial.println("Reached destination.");
        initiateLanding();  // Land at destination
        break;
      } else {
        float bearing = calculateBearing(currentLatitude, currentLongitude, targetLat, targetLon);
        adjustFlightForTarget(bearing, targetAltitude);
      }
    }
    delay(100);  // Adjust the delay to control flight frequency
  }
}

void adjustFlightForTarget(float targetBearing, float targetAltitude) {
  float yawCorrection = calculatePID(targetBearing, yawInput, prevYawError, yawIntegral, kp_yaw, ki_yaw, kd_yaw);
  float altitudeCorrection = calculatePID(targetAltitude, currentAltitude, prevAltitudeError, altitudeIntegral, kp_pitch, ki_pitch, kd_pitch);

  if (xSemaphoreTake(motorMutex, portMAX_DELAY)) {
    setMotorSpeeds(throttleInput + altitudeCorrection, 0, 0, yawCorrection);
    xSemaphoreGive(motorMutex);
  }
}

void initiateLanding() {
  int landingThrottle = throttleInput;
  while (landingThrottle > 1000) {
    landingThrottle -= 10;  // Gradually reduce throttle
    if (xSemaphoreTake(motorMutex, portMAX_DELAY)) {
      setMotorSpeeds(landingThrottle, 0, 0, 0);
      xSemaphoreGive(motorMutex);
    }
    delay(100);  // Delay to simulate controlled descent
  }
}
