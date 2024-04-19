
#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

#include "types.h"
#include "drivetrain_config.h"
#include "utils.h"
#include "pid.hpp"

namespace WAYPOINTS {
    using namespace COMMON;
    using namespace pimoroni;
    class WaypointNavigation {
    public:
        WaypointNavigation();
        ~WaypointNavigation();
        void navigate(const VehicleState& currentState); //update the desired movement to get to the next waypoint
        float desiredV;  // desired velocity to get to next waypoint
        float desiredW;  // desired angular velocity to get to next waypoint
        uint8_t targetWaypointIndex = 0; // the index of the current waypoint we're navigating to
        Waypoint targetWaypoint; // the actual waypoint we're navigating to
        uint8_t nextWaypoint(const uint8_t currentWaypointIndex, const VehicleState& currentState); //find the index of the next waypoint to navigate to
        uint8_t nearestWaypointIndex = 0; // the index of the waypoint we're closest to
        uint8_t nearestWaypoint(const VehicleState& currentState); //find the index of the closest waypoint
        float headingToWaypoint(const Waypoint& target, const VehicleState& currentState); // heading to a waypoint. not currently used?
        float bearingToWaypoint(const Waypoint& target, const VehicleState& currentState); // Compass bearing to a waypoint
        float distanceToWaypoint(const Waypoint& target, const VehicleState& currentState); // distance to a waypoint
        float lookAhead = 0.2; //lookahead distance in metres
        float maxTurnVelocity = 10; //max turn velocity in radians per second
        float unwrapHeading(const float targetHeading, float currentHeading); //find "nearest" description of current heading to target

    private:
        float headingPGain = 20;
        float headingIGain = 0.5;
        float headingDGain = 0.0;
        float UPDATE_RATE = 0.02; //seconds

        PID headingPID = PID(headingPGain, headingIGain, headingDGain, UPDATE_RATE); // used for steering to waypoints
    };
}
#endif // WAYPOINT_NAVIGATION_H