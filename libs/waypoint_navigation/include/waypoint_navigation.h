
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
        void navigate(const State& currentState); //update the desired movement to get to the next waypoint
        float desiredV;  // desired velocity to get to next waypoint
        float desiredW;  // desired angular velocity to get to next waypoint
        const static uint8_t waypointBufferSize = 40;
        Waypoint waypointBuffer[waypointBufferSize]; // waypoint buffer
        void addWaypoint(const Waypoint& newWaypoint); // add a waypoint to the buffer
        bool isWaypointEmpty(const Waypoint& waypoint); // check if a waypoint slot in the buffer is empty
        uint8_t targetWaypointIndex; // the index of the current waypoint we're navigating to
        Waypoint targetWaypoint; // the actual waypoint we're navigating to
        uint8_t nextWaypoint(const uint8_t currentWaypointIndex, const State& currentState); //find the index of the next waypoint to navigate to
        uint8_t nearestWaypointIndex; // the index of the waypoint we're closest to
        uint8_t nearestWaypoint(const State& currentState); //find the index of the closest waypoint
        void clearWaypointBuffer(); // clear all waypoints from buffer
        float headingToWaypoint(const Waypoint& target, const State& currentState); // heading to a waypoint. not currently used?
        float bearingToWaypoint(const Waypoint& target, const State& currentState); // Compass bearing to a waypoint
        float distanceToWaypoint(const Waypoint& target, const State& currentState); // distance to a waypoint
        float lookAhead = 0.2; //lookahead distance in metres
        float maxTurnVelocity = 1; //max turn velocity in radians per second
        float unwrapHeading(const float targetHeading, float currentHeading); //find "nearest" description of current heading to target

    private:
        float headingPGain = 1;
        float headingIGain = 0;
        float headingDGain = 0;
        float UPDATE_RATE = 0.02; //seconds
        Waypoint nanWaypoint = {NAN, NAN, NAN, NAN}; // Create a NaN waypoint

        PID headingPID = PID(headingPGain, headingIGain, headingDGain, UPDATE_RATE); // used for steering to waypoints
    };
}
#endif // WAYPOINT_NAVIGATION_H