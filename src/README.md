Components:

* ESP32 Dev Module with dual CPU cores, three UART ports and two I2C ports.

* GPS Module (e.g., NEO-6M): Provides the drone's location (latitude, longitude, altitude).

* UART Receiver (e.g., FlySky or FrSky): To receive signals for throttle, pitch, roll, yaw from the remote control.

* IMU Sensor: For gyroscope and accelerometer readings. STEMMA QT MPU-6050 using the Adafruit MPU6050 library, the process involves integrating the Adafruit MPU6050 library and modifying the code to use its functions. The Adafruit library is designed for simplicity and works well with the STEMMA QT I2C connector.

* Brushless Motors (4): For quadcopters.

* Electronic Speed Controllers (ESCs): To control the speed of the motors. The ESCs are controlled using PWM signals.
 
* LiPo Battery: For powering the drone.

* Propellers: For lift.

* PDB (Power Distribution Board): To distribute power from the battery.

* Remote Control: You can use a radio transmitter/receiver to control throttle, pitch, roll, and yaw remotely.

* Arduino v3.0.0 with FreeRTOS and ESP32 core 3.0.5
