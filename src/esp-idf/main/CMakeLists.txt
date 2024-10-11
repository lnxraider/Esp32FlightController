idf_component_register(SRCS "main.cpp" "Communication.cpp" "Drone.cpp" "FlightControl.cpp" "MotorControl.cpp" "PID.cpp" 
	INCLUDE_DIRS "." "../../components/Bolder_Flight_Systems_SBUS/src/" "../../components/Adafruit_MPU6050/" "../../components/Adafruit_BMP280_Library/" "../../components/TinyGPSPlus/src/" 
		    REQUIRES arduino Bolder_Flight_Systems_SBUS Adafruit_MPU6050 Adafruit_BMP280_Library TinyGPSPlus)
