ChatGPT generated Flight Controller for the ESP32 Dev Module

Breakdown of the Complete Code:
1. MPU6050 Initialization and Setup:
   * The MPU6050 sensor is initialized using the Adafruit library. The accelerometer and gyroscope ranges are set.
   * The sensor data is used to calculate real-time pitch, roll, and yaw, which are critical for stabilizing the drone.

2. GPS Initialization and Data Handling:
   * The TinyGPS++ library is used to decode GPS data received from a GPS module connected to Serial1.
   * GPS data (latitude, longitude, altitude) is read and stored in shared variables, protected by a mutex to ensure thread safety.

3. Flight Control Task (flightControlTask()):
   * PID Controller: This task uses a PID controller to stabilize the drone. It compares the desired input (e.g., from a controller or UART command) with the current sensor data to generate corrective actions.
   * Mutex for Motor Control: A mutex (motorMutex) is used to ensure that the motor speeds are safely updated without interference from other tasks.

4. Motor Control (setMotorSpeeds()):
   * Motor speeds are calculated based on throttle, pitch, roll, and yaw PID corrections.
   * The analogWrite() function is used to send PWM signals to the motor control pins to adjust the motor speeds.

5. Communication Task (communicationTask()):
   * GPS Data: This task continuously reads data from the GPS module and updates the shared GPS data variables.
   * Command Handling: It also handles incoming UART commands from the ground station or remote control, adjusting the drone's flight parameters (e.g., throttle) in real-time.

6. Mutex Usage:
   * The motorMutex ensures that only one task at a time can update motor speeds, avoiding potential conflicts with shared resources.
   * The gpsMutex is used to protect access to the shared GPS data, ensuring that both the communicationTask() and any other tasks that need GPS data can safely access it.

Conclusion:
 This code demonstrates how to combine multiple tasks in a FreeRTOS environment on the ESP32, utilizing mutexes to ensure safe communication between tasks. It integrates GPS navigation, flight stabilization using MPU6050, and motor control, all while ensuring that multiple tasks can run concurrently without conflicts.


