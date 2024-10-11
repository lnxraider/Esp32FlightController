1. Modular Class-Based Architecture
   * Single Responsibility Principle: Each class is responsible for one specific function of the drone (e.g., motor control, flight control, communication, battery monitoring).
   * Separation of Concerns: Tasks such as sensor management, motor control, and flight logic are split across individual classes to ensure clean and modular code.
   * Reuse and Extensibility: This structure allows for easier extension of functionality (e.g., adding new sensors or expanding motor control) without impacting unrelated parts of the code.

2. Centralized SBUS Management
   * SBUS Centralization: The Communication class manages all SBUS interactions and data collection, providing a single source of truth for the receiver input.
   * Accessor Methods: Classes that require SBUS data (e.g., FlightControl) obtain it through accessor methods in the Communication class, promoting data consistency and reducing redundancy.

3. PID-Controlled Flight Management
   * PID Controllers for Flight: FlightControl manages flight dynamics using Proportional-Integral-Derivative (PID) controllers for pitch, roll, and yaw.
   * Dynamic PID Tuning: The system allows real-time tuning of PID parameters via SBUS channels, enabling on-the-fly adjustment during flight.

4. FreeRTOS Task Management
   * Real-Time Task Scheduling: The drone system uses FreeRTOS to manage concurrent tasks, ensuring that flight control, communication, and sensor updates run concurrently.
   * Task Prioritization: Critical tasks like flight control are given higher priority, while sensor updates and communication tasks are lower priority.

5. Sensor Integration
   * Modular Sensor Management: The Communication class is responsible for integrating the BMP280 sensor (for altitude) and MPU6050 (for IMU data), along with GPS data using TinyGPS++.
   * Data Updates in Real-Time: Sensors are updated in real-time through dedicated methods in the Communication class, ensuring timely data for control algorithms.

6. Motor Control and PWM Management
   * Motor Thrust Control: The MotorControl class handles the logic for adjusting motor speeds based on the control inputs calculated by FlightControl.
   * PWM Signal Output: The motor outputs are controlled using PWM signals with a configurable frequency and range, ensuring precise control over motor speeds.

7. Battery Monitoring and Safety
   * Voltage Monitoring: The BatteryMonitor class monitors battery voltage using analog input.
   * Failsafe Handling: If the voltage drops below a threshold, failsafe actions (such as landing or returning home) can be triggered to ensure safety during flight.

8. Error Handling and Initialization
   * Robust Initialization: Each class (e.g., Communication, MotorControl) is initialized separately, with proper error handling to ensure all hardware components are functioning before the flight starts.
   * Graceful Failures: If any sensor or component fails to initialize, the system halts and reports the error to the user.

9. Real-Time Sensor Fusion
   * IMU Data Integration: The IMU (MPU6050) data is integrated with PID control in FlightControl to maintain stable flight, using roll, pitch, and yaw measurements.
   * Altitude Measurement: The BMP280 sensor is used to measure the altitude and integrate it into the control logic or navigation features.

10. GPS Navigation Support
    * TinyGPS++ Library: The GPS data (latitude, longitude, etc.) is processed using TinyGPS++ within the Communication class.
    * Future Expansion for GPS Navigation: The code is designed to easily expand for more complex GPS-based flight modes like Return-to-Home, Waypoint Navigation, etc.

11. Scalability and Extensibility
    * Additional Sensors and Components: The architecture supports the addition of new sensors (e.g., LiDAR, barometers) with minimal modification to existing code.
    * Flight Mode Expansion: New flight modes or control systems (e.g., altitude hold, position hold) can be added by extending the FlightControl class or introducing new control systems.

12. Hardware Abstraction and Flexibility
    * Pin and Serial Configuration: All hardware connections (e.g., motor control, SBUS, GPS) are abstracted and can be easily reconfigured without deep changes to the codebase.
    * Flexible UART and I2C Communication: The use of configurable serial objects (e.g., serialGPS) and I2C communication allows the code to be adapted to different ESP32 setups and peripherals.

