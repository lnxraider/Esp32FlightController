#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

#include "PID.h"
#include "Communication.h"
#include <cmath>
#include "Drone.h"  // Assuming the other Drone components (MotorControl, PID, etc.) are included here

enum class FlightMode {
    IDLE,
    ARMING,
    TAKEOFF,
    MANUAL,
    ALTITUDE_HOLD,
    RETURN_TO_HOME,
    LANDING,
    EMERGENCY
    AUTONOMOUS
};

struct Waypoint {
    float latitude;
    float longitude;
    float altitude;
    float yaw;  // Optional: heading at waypoint
};

class FlightControl {
public:
FlightControl(Communication& comm, MotorControl& motorControl);

    void mapReceiverInput();
    void tunePID();

    float getThrottle();
    float getRoll();
    float getPitch();
    float getYaw();

    void init();
    void update();
    void enableReturnToHome();
    bool isAtHome();
    void calculateBearingAndDistance();
    void setFlightMode(FlightMode mode);

    // Autonomous flight
    void setWaypoints(const std::vector<Waypoint>& waypoints);
    void startAutonomousMission();
    void pauseAutonomousMission();
    void resumeAutonomousMission();
    void abortAutonomousMission();

    // Obstacle avoidance
    virtual bool detectObstacle();
    virtual void avoidObstacle();

private:
    static constexpr float MIN_SBUS_VALUE = 172.0f;
    static constexpr float MAX_SBUS_VALUE = 1811.0f;
    static constexpr float HOME_THRESHOLD_DISTANCE = 2.0f; // meters

    static const float KP_VL = 1.0;
    static const float KI_VL = 0.1;
    static const float KD_VL = 0.05;

    FlightMode currentMode = FlightMode::MANUAL;

    PID pidThrottle;
    PID pidRoll;
    PID pidPitch;
    PID pidYaw;

    Communication& communication;
    MotorControl& motors;

    float throttle;
    float roll;
    float pitch;
    float yaw;

    // Desired setpoints (e.g., from receiver inputs)
    float desiredThrottle;
    float desiredRoll;
    float desiredPitch;
    float desiredYaw;

    // Home position
    float homeLatitude;
    float homeLongitude;

    // Current distance to home and bearing
    float distanceToHome;
    float bearingToHome;

    // Flags
    bool returnToHomeActive = false;
    
    // Helper functions
    float calculateDistance(float lat1, float lon1, float lat2, float lon2);
    float calculateBearing(float lat1, float lon1, float lat2, float lon2);

    // RTH logic
    void returnToHome();

    // Validate GPS Coordinates
    bool isValidGPSCoordinate(float lat, float lon);

    // Flight Modes
    inline bool initiateTakeoff() {return true;}
    inline bool initiateReturnToHome() {return true;}
    inline bool initiateEmergencyLanding() {return true;}

    // Autonomous Flight
    std::vector<Waypoint> missionWaypoints;
    size_t currentWaypointIndex;
    void navigateToWaypoint(const Waypoint& waypoint);
    bool isWaypointReached(const Waypoint& waypoint);
    void updateAutonomousNavigation();
};

#endif  // FLIGHTCONTROL_H

