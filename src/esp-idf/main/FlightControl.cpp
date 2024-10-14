#include "FlightControl.h"
#include <Arduino.h>

FlightControl::FlightControl(Communication& comm, MotorControl& motorControl)
    : communication(comm),
      motors(motorControl),
      pidThrottle(KP_VL, KI_VL, KD_VL),
      pidRoll(KP_VL, KI_VL, KD_VL),
      pidPitch(KP_VL, KI_VL, KD_VL),
      pidYaw(KP_VL, KI_VL, KD_VL)
{
    // Initialize PIDs if necessary
}

void FlightControl::mapReceiverInput() {
    bfs::SbusData sbusData = communication->getSbusData();

    // Map SBUS channels to control inputs
    desiredThrottle = map(sbusData.ch(THR), MIN_SBUS_VALUE, MAX_SBUS_VALUE, 0, 1);  // Normalize to 0-1
    desiredRoll     = map(sbusData.ch(AIL), MIN_SBUS_VALUE, MAX_SBUS_VALUE, -1, 1); // Normalize to -1 to 1
    desiredPitch    = map(sbusData.ch(ELE), MIN_SBUS_VALUE, MAX_SBUS_VALUE, -1, 1); // Normalize to -1 to 1
    desiredYaw      = map(sbusData.ch(RUD), MIN_SBUS_VALUE, MAX_SBUS_VALUE, -1, 1); // Normalize to -1 to 1
}

void FlightControl::tunePID() {
    // Dynamic PID tuning via SBUS channels (if desired)
    bfs::SbusData sbusData = communication->getSbusData();

    float kp = map(sbusData.ch(4), MIN_SBUS_VALUE, MAX_SBUS_VALUE, 0.0, 5.0);  // Channel 5 for Kp
    float ki = map(sbusData.ch(5), MIN_SBUS_VALUE, MAX_SBUS_VALUE, 0.0, 1.0);  // Channel 6 for Ki
    float kd = map(sbusData.ch(6), MIN_SBUS_VALUE, MAX_SBUS_VALUE, 0.0, 1.0);  // Channel 7 for Kd

    pidRoll.setTunings(kp, ki, kd);
    pidPitch.setTunings(kp, ki, kd);
    pidYaw.setTunings(kp, ki, kd);
}

float FlightControl::getThrottle() {
    // For throttle, we might bypass PID for direct control
    return desiredThrottle;
}

float FlightControl::getRoll() {
    // Use PID to calculate the necessary roll correction
    float currentRoll = communication->getIMUData().roll;
    roll = pidRoll.calculate(desiredRoll, currentRoll);
    return roll;
}

float FlightControl::getPitch() {
    float currentPitch = communication->getIMUData().pitch;
    pitch = pidPitch.calculate(desiredPitch, currentPitch);
    return pitch;
}

float FlightControl::getYaw() {
    float currentYaw = communication->getIMUData().yaw;
    yaw = pidYaw.calculate(desiredYaw, currentYaw);
    return yaw;
}

// Function to initialize the home position
void FlightControl::init() {
    // Record home position (current GPS coordinates)
    homeLatitude = communication->getGPSLatitude();
    homeLongitude = communication->getGPSLongitude();
}

void FlightControl::update() {
    // If signal is lost, trigger RTH
    if (communication->isSignalLost()) {
        enableReturnToHome();
        setFlightMode(FlightMode::RETURN_TO_HOME);
    }

    switch (currentMode) {
        case FlightMode::MANUAL:
            mapReceiverInput();
            tunePID();
            break;
        case FlightMode::RETURN_TO_HOME:
            calculateBearingAndDistance();
            returnToHome();
            break;
        case FlightMode::LANDING:
            motors.land();
            //updateLanding();
            break;
        case FlightMode::EMERGENCY:
            motors.land();
            //performEmergencyProcedure();
            break;
        case FlightMode::AUTONOMOUS:
            updateAutonomousNavigation();
            break;
    }
}

// Enable Return to Home
void FlightControl::enableReturnToHome() {
    returnToHomeActive = true;
}

// Check if the drone has reached the home location
bool FlightControl::isAtHome() {
    return distanceToHome < HOME_THRESHOLD_DISTANCE;  // Threshold distance in meters
}

// Calculate the bearing and distance to the home position
void FlightControl::calculateBearingAndDistance() {
    float currentLat = communication->getGPSLatitude();
    float currentLon = communication->getGPSLongitude();
    
    if (!isValidGPSCoordinate(currentLat, currentLon)) {
        // Handle invalid GPS data
        return;
    }

    // Calculate distance and bearing to home
    distanceToHome = calculateDistance(currentLat, currentLon, homeLatitude, homeLongitude);
    bearingToHome = calculateBearing(currentLat, currentLon, homeLatitude, homeLongitude);
}

// Return to Home logic
void FlightControl::returnToHome() {
    float currentAltitude = communication.getAltitude();
    float rthAltitude = max(currentAltitude, MIN_RTH_ALTITUDE);

    if (isAtHome()) {
        // Landing procedure
        motors.land();
        returnToHomeActive = false;
    } else {
        // First, ascend to safe RTH altitude
        if (currentAltitude < rthAltitude) {
           ascendTo(rthAltitude);
        }
        // Adjust drone's bearing and move towards home
        motors.adjustBearing(bearingToHome);
        motors.adjustThrottleForDistance(distanceToHome);
    }
}

// Helper function to calculate distance between two GPS points
float FlightControl::calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    const double R = 6371e3; // Radius of Earth in meters
    float phi1 = lat1 * M_PI / 180;
    float phi2 = lat2 * M_PI / 180;
    float deltaPhi = (lat2 - lat1) * M_PI / 180;
    float deltaLambda = (lon2 - lon1) * M_PI / 180;

    float a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
              cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c;
}

// Helper function to calculate bearing from one GPS point to another
float FlightControl::calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float phi1 = lat1 * M_PI / 180;
    float phi2 = lat2 * M_PI / 180;
    float deltaLambda = (lon2 - lon1) * M_PI / 180;

    float y = sin(deltaLambda) * cos(phi2);
    float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
    float bearing = atan2(y, x);

    return fmod((bearing * 180 / M_PI) + 360, 360); // Convert bearing to degrees
}

bool FlightControl::isValidGPSCoordinate(float lat, float lon) {
    return (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0);
}

// Set the Flight Mode
void FlightControl::setFlightMode(FlightMode newMode) {
    if (newMode == currentMode) return;

    switch (newMode) {
        case FlightState::IDLE:
             // Check for arming command
             break;
        case FlightMode::ARMING:
            if (!motors.arm()) {
                // Handle arming failure
                return;
            }
            break;
        case FlightMode::TAKEOFF:
            if (currentMode != FlightMode::ARMING) {
                // Can only take off from ARMING mode
                return;
            }
            initiateTakeoff();
            break;
        case FlightMode::RETURN_TO_HOME:
            initiateReturnToHome();
            break;
        case FlightMode::EMERGENCY:
            initiateEmergencyLanding();
            break;
        // ... handle other mode transitions
    }

    currentMode = newMode;
}

void FlightControl::setWaypoints(const std::vector<Waypoint>& waypoints) {
    missionWaypoints = waypoints;
    currentWaypointIndex = 0;
}

void FlightControl::startAutonomousMission() {
    if (!missionWaypoints.empty()) {
        setFlightMode(FlightMode::AUTONOMOUS);
        currentWaypointIndex = 0;
    }
}

void FlightControl::updateAutonomousNavigation() {
    if (currentWaypointIndex < missionWaypoints.size()) {
        const Waypoint& currentWaypoint = missionWaypoints[currentWaypointIndex];

        if (isWaypointReached(currentWaypoint)) {
            currentWaypointIndex++;
            if (currentWaypointIndex >= missionWaypoints.size()) {
                // Mission complete, initiate landing or RTH
                setFlightMode(FlightMode::LANDING);
                return;
            }
        }

        navigateToWaypoint(currentWaypoint);
    }
}

void FlightControl::navigateToWaypoint(const Waypoint& waypoint) {
    float currentLat = communication.getGPSLatitude();
    float currentLon = communication.getGPSLongitude();
    float currentAlt = communication.getAltitude();

    // Calculate desired pitch, roll, yaw, and throttle to reach the waypoint
    float distanceToWaypoint = calculateDistance(currentLat, currentLon, waypoint.latitude, waypoint.longitude);
    float bearingToWaypoint = calculateBearing(currentLat, currentLon, waypoint.latitude, waypoint.longitude);

    // Use PID controllers to generate control inputs
    float desiredPitch = pitchPID.calculate(0, -distanceToWaypoint * cos(bearingToWaypoint));
    float desiredRoll = rollPID.calculate(0, -distanceToWaypoint * sin(bearingToWaypoint));
    float desiredYaw = yawPID.calculate(waypoint.yaw, communication.getIMUData().yaw);
    float desiredThrottle = altitudePID.calculate(waypoint.altitude, currentAlt);

    // Check for obstacles
    if (detectObstacle()) {
        // Implement obstacle avoidance algorithm
        avoidObstacle();
    } else {
        // Apply the calculated control inputs
        motors.controlMotors(desiredThrottle, desiredRoll, desiredPitch, desiredYaw);
    }
}

bool FlightControl::isWaypointReached(const Waypoint& waypoint) {
    float currentLat = communication.getGPSLatitude();
    float currentLon = communication.getGPSLongitude();
    float currentAlt = communication.getAltitude();

    float distanceToWaypoint = calculateDistance(currentLat, currentLon, waypoint.latitude, waypoint.longitude);
    float altitudeDifference = abs(currentAlt - waypoint.altitude);

    // Define thresholds for considering a waypoint reached
    const float DISTANCE_THRESHOLD = 2.0; // meters
    const float ALTITUDE_THRESHOLD = 1.0; // meters

    return (distanceToWaypoint < DISTANCE_THRESHOLD) && (altitudeDifference < ALTITUDE_THRESHOLD);
}

bool FlightControl::detectObstacle() {
    // Implement obstacle detection using sensors
    // Return true if an obstacle is detected within a safety threshold
    return false;
}

void FlightControl::avoidObstacle() {
    // Implement obstacle avoidance algorithm
    // This could involve changing altitude, adjusting course, or hovering in place
    return;
}
