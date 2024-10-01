#include <Arduino.h>
#include <sbus.h>                     // SBUS library for FrSky receiver
#include <Adafruit_MPU6050.h>          // Adafruit MPU6050 library
#include <Adafruit_Sensor.h>           // Adafruit unified sensor library
#include <Wire.h>                      // For I2C communication with MPU6050
#include <TinyGPSPlus.h>               // GPS parsing library
#include <HardwareSerial.h>            // UART for GPS communication
#include <freertos/FreeRTOS.h>         // FreeRTOS for task management
#include <freertos/task.h>             // FreeRTOS task handling
#include <freertos/semphr.h>           // FreeRTOS semaphore handling

#define GPS_RX_PIN 18
#define GPS_TX_PIN 19
#define SBUS_RX_PIN 16
#define SBUS_TX_PIN 17

// Define receiver channels
#define AIL 0         // Aileron/Roll
#define ELE 1         // Elevator/Pitch
#define THR 2         // Throttle
#define RUD 3         // Rudder/Yaw

// --- SBUS Receiver and MPU6050 initialization ---

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, SBUS_RX_PIN, SBUS_TX_PIN, true);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial2, SBUS_RX_PIN, SBUS_TX_PIN,  true);
/* SBUS data */
bfs::SbusData data;

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;                        // GPS object
HardwareSerial serialGPS(1);            // GPS on Serial1 (TX = GPIO17)

// --- FreeRTOS task handles
TaskHandle_t FlightControlTask;
TaskHandle_t CommunicationTask;

// --- Global Variables ---
SemaphoreHandle_t motorMutex;
SemaphoreHandle_t gpsMutex;

// --- PID Constants ---
float kp_roll = 0.5, ki_roll = 0.05, kd_roll = 0.1;
float kp_pitch = 0.5, ki_pitch = 0.05, kd_pitch = 0.1;
float kp_yaw = 0.5, ki_yaw = 0.05, kd_yaw = 0.1;

// --- Error Variables ---
float rollError = 0, pitchError = 0, yawError = 0;
float prevRollError = 0, prevPitchError = 0, prevYawError = 0;
float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;

// --- SBUS Channel Data ---
uint16_t throttleInput = 1000;
uint16_t pitchInput = 1500, rollInput = 1500, yawInput = 1500;

// --- GPS Variables ---
double currentLatitude = 0.0;
double currentLongitude = 0.0;
double currentAltitude = 0.0;
double homeLatitude = 0.0;
double homeLongitude = 0.0;

// --- Battery Monitoring ---
float batteryVoltage = 0.0;
float lowBatteryThreshold = 3.5;

// --- Constants ---
#define MOTOR1_PIN 5
#define MOTOR2_PIN 6
#define MOTOR3_PIN 7
#define MOTOR4_PIN 8
#define BATTERY_PIN 36  // ADC pin for battery voltage monitoring

void setup() {
  Serial.begin(115200);
  serialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // Initialize GPS UART

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 initialized and configured");

  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();

  // Initialize mutexes
  motorMutex = xSemaphoreCreateMutex();
  gpsMutex = xSemaphoreCreateMutex();

  // Create tasks for each core
  xTaskCreatePinnedToCore(
    flightControlTask,    // Task function
    "Flight Control",     // Task name
    10000,                // Stack size
    NULL,                 // Parameters (none)
    1,                    // Priority (higher for flight control)
    &FlightControlTask,   // Task handle
    1                     // Core 1 (real-time flight control)
  );

  xTaskCreatePinnedToCore(
    communicationTask,    // Task function
    "Communication",      // Task name
    10000,                // Stack size
    NULL,                 // Parameters (none)
    1,                    // Priority
    &CommunicationTask,   // Task handle
    0                     // Core 0 (for GPS and receiver)
  );
}

void loop() {
  monitorBattery();
}

// ----------------------- FLIGHT CONTROL TASK -----------------------
void flightControlTask(void *pvParameters) {
  while (true) {
    sensors_event_t accelEvent, gyroEvent, tempEvent;

    // Read MPU6050 sensor data
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    float pitch = gyroEvent.gyro.x * 57.2958; // Convert rad/s to degrees/s
    float roll  = gyroEvent.gyro.y * 57.2958;
    float yaw   = gyroEvent.gyro.z * 57.2958;

    // Update SBUS channels
    sbusProcess();

    // Calculate PID corrections
    float rollCorrection = calculatePID(rollInput - 1500, roll, prevRollError, rollIntegral, kp_roll, ki_roll, kd_roll);
    float pitchCorrection = calculatePID(pitchInput - 1500, pitch, prevPitchError, pitchIntegral, kp_pitch, ki_pitch, kd_pitch);
    float yawCorrection = calculatePID(yawInput - 1500, yaw, prevYawError, yawIntegral, kp_yaw, ki_yaw, kd_yaw);

    // Safe motor control using mutex
    if (xSemaphoreTake(motorMutex, portMAX_DELAY)) {
      setMotorSpeeds(throttleInput, pitchCorrection, rollCorrection, yawCorrection);
      xSemaphoreGive(motorMutex);
    }

    delay(20);  // Control loop at 50Hz
  }
}

void setMotorSpeeds(int throttle, float pitchCorrection, float rollCorrection, float yawCorrection) {
  int motor1Speed = throttle + pitchCorrection + rollCorrection + yawCorrection;
  int motor2Speed = throttle + pitchCorrection - rollCorrection - yawCorrection;
  int motor3Speed = throttle - pitchCorrection + rollCorrection - yawCorrection;
  int motor4Speed = throttle - pitchCorrection - rollCorrection + yawCorrection;

  motor1Speed = constrain(motor1Speed, 1000, 2000);
  motor2Speed = constrain(motor2Speed, 1000, 2000);
  motor3Speed = constrain(motor3Speed, 1000, 2000);
  motor4Speed = constrain(motor4Speed, 1000, 2000);

  analogWrite(MOTOR1_PIN, motor1Speed);
  analogWrite(MOTOR2_PIN, motor2Speed);
  analogWrite(MOTOR3_PIN, motor3Speed);
  analogWrite(MOTOR4_PIN, motor4Speed);
}

// ----------------------- COMMUNICATION TASK -----------------------
void communicationTask(void *pvParameters) {
  while (true) {
    // GPS data handling
    while (serialGPS.available() > 0) {
      char c = serialGPS.read();
      gps.encode(c);

      if (gps.location.isUpdated()) {
        if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
          currentLatitude = gps.location.lat();
          currentLongitude = gps.location.lng();
          currentAltitude = gps.altitude.meters();
          xSemaphoreGive(gpsMutex);
        }
      }
    }

    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      handleCommand(command);
    }

    delay(100);  // 10Hz communication task
  }
}

// ------------------- SBUS PROCESSING FUNCTION -------------------
void sbusProcess() {
  if (sbus_rx.Read()) {
    /* Grab the received data */
    data = sbus_rx.data();

    throttleInput = map(data.ch[THR], 172, 1811, 1000, 2000);  // Channel 3 for throttle
    pitchInput    = map(data.ch[ELE], 172, 1811, 1000, 2000);  // Channel 2 for pitch
    rollInput     = map(data.ch[AIL], 172, 1811, 1000, 2000);  // Channel 1 for roll
    yawInput      = map(data.ch[RUD], 172, 1811, 1000, 2000);  // Channel 4 for yaw
  }
}

// ------------------ HELPER FUNCTIONS ------------------

// PID Controller for motor correction
float calculatePID(float target, float current, float &prevError, float &integral, float kp, float ki, float kd) {
  float error = target - current;
  integral += error;
  float derivative = error - prevError;
  prevError = error;
  return (kp * error) + (ki * integral) + (kd * derivative);
}

// Monitor battery voltage and trigger return-to-home if needed
void monitorBattery() {
  batteryVoltage = analogRead(BATTERY_PIN) * (3.3 / 4095.0) * 2;  // Adjust based on battery divider

  if (batteryVoltage < lowBatteryThreshold) {
    Serial.println("Low battery! Initiating return to home.");
    automatedFlight(homeLatitude, homeLongitude, currentAltitude);
  }
}

// Automated flight to a GPS coordinate
void automatedFlight(double targetLat, double targetLon, double targetAltitude) {
  while (true) {
    if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
      double distanceToTarget = calculateDistance(currentLatitude, currentLongitude, targetLat, targetLon);
      xSemaphoreGive(gpsMutex);

      if (distanceToTarget < 2.0) {  // Target reached within 2 meters
        Serial.println("Arrived at target location.");
        break;
      }

      // Adjust flight to target coordinates
      float bearingToTarget = calculateBearing(currentLatitude, currentLongitude, targetLat, targetLon);
      adjustCourse(bearingToTarget, targetAltitude);
    }

    delay(100);  // Adjust flight every 100 ms
  }
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  // Haversine formula to calculate distance between two lat/lon pairs
  // Simplified for readability
  return TinyGPSPlus::distanceBetween(lat1, lon1, lat2, lon2);
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  return TinyGPSPlus::courseTo(lat1, lon1, lat2, lon2);
}

void adjustCourse(float targetBearing, float targetAltitude) {
  // Adjust yaw, pitch, and altitude accordingly
  yawInput = map(targetBearing, 0, 360, 1000, 2000);
  throttleInput = map(targetAltitude, 0, 100, 1000, 2000);
}

// Handle incoming serial commands
void handleCommand(String command) {
  if (command == "SET_HOME") {
    if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
      homeLatitude = currentLatitude;
      homeLongitude = currentLongitude;
      xSemaphoreGive(gpsMutex);
      Serial.println("Home location set.");
    }
  }
}
