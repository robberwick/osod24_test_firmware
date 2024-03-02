#include <cstdio>
#include <math.h>
#include <algorithm>
#include "types.h"
#include "drivetrain_config.h"
#include "waypoint_navigation.h"

namespace WAYPOINTS {

WaypointNavigation::WaypointNavigation(){
    Waypoint lavaWaypoints[43] = {   ///exmple waypoint list for testing
        {0.000, 0.000,  0.000, 0.100},
        {0.000, 0.000,  0.000, 0.1},
        {0.000, 0.078,  0.000, 0.1},
        {0.000, 0.235,  0.000, 0.100},
        {0.000, 0.435,  0.000, 0.100},
        {0.000, 0.635,  0.000, 0.100},
        {0.000, 0.835,  0.000, 0.100},
        {0.000, 1.035,  0.000, 0.1000},
        {0.000, 1.235,  0.000, 0.100},
        {0.000, 1.435,  0.000, 0.1000},
        {0.000, 1.635,  -0.017, 0.100},
        {-0.003, 1.835,  -0.140, 0.1000},
        {-0.031, 2.033,  -0.332, 0.100},
        {-0.096, 2.223,  -0.489, 0.1000},
        {-0.190, 2.399,  -0.646, 0.100},
        {-0.311, 2.559,  -0.555, 0.1000},
        {-0.416, 2.729,  -0.396, 0.100},
        {-0.493, 2.913,  -0.237, 0.1000},
        {-0.540, 3.108,  -0.075, 0.100},
        {-0.555, 3.307,  0.000, 0.1000},
        {-0.555, 3.507,  0.000, 0.100},
        {-0.555, 3.707,  0.017, 0.1000},
        {-0.552, 3.907,  0.087, 0.100},
        {-0.534, 4.106,  0.192, 0.1000},
        {-0.496, 4.303,  0.419, 0.100},
        {-0.415, 4.485,  0.541, 0.1000},
        {-0.312, 4.657,  0.611, 0.100},
        {-0.197, 4.821,  0.524, 0.1000},
        {-0.097, 4.994,  0.314, 0.100},
        {-0.035, 5.184,  0.105, 0.1000},
        {-0.014, 5.383,  0.000, 0.100},
        {-0.014, 5.583,  0.000, 0.1000},
        {-0.014, 5.783,  0.000, 0.100},
        {-0.014, 5.983,  0.000, 0.1000},
        {-0.014, 6.183,  0.000, 0.100},
        {-0.014, 6.383,  0.000, 0.1000},
        {-0.014, 6.583,  0.000, 0.100},
        {-0.014, 6.783,  0.000, 0.1000},
        {-0.014, 6.983,  0.000, 0.100},
        {-0.014, 7.183,  0.000, 0.1},
        {-0.038, 7.4,  0.000, 0.0},
        {-0.038, 7.5,  0.000, 0.0},
        {-0.023, 7.6,  0.000, 0.0}
    };
    size_t i;
    size_t numWaypoints = sizeof(lavaWaypoints) / sizeof(lavaWaypoints[0]);
    for (i = 0; i < numWaypoints && i < waypointBufferSize; i++) {
        waypointBuffer[i] = lavaWaypoints[i];
    }

    // Initialize remaining waypoints in the buffer to NaN
   
    for (; i < waypointBufferSize; i++) {
        waypointBuffer[i] = nanWaypoint;
    }
}


void WaypointNavigation::navigate(const VehicleState& currentState) {
    // updates desiredV and desiredW (speed and turn velocity)
    // based on the current position and the list of waypoints.
    // the velocity comes from the speed associated with the closest waypoint
    // the turn velocity comes from PID feedback on the heading to the next waypoint

    // find nearest waypoint and use it to set the speed
    nearestWaypointIndex = nearestWaypoint(currentState); //TODO: check if UINT8_MAX?
    Waypoint nearestWaypoint = waypointBuffer[nearestWaypointIndex];
    if (isWaypointEmpty(nearestWaypoint)){
        // if the nearest waypoint is empty or doesn't have a speed assigned, stop
        desiredV = 0;
    } else {
        desiredV = nearestWaypoint.speed;
    }

    // find target waypoint and use it to set the angular velocity
    targetWaypointIndex = nextWaypoint(targetWaypointIndex, currentState);
    targetWaypoint = waypointBuffer[targetWaypointIndex];
    if (isWaypointEmpty(targetWaypoint)){
        // if (somehow!?) the target waypoint is empty, stop
        desiredW = 0;
        desiredV = 0;
        printf("stopped, on last waypoint");
    } else {
        float bearingToNextWaypoint = bearingToWaypoint(targetWaypoint, currentState);

        // reframe current heading to be described in a way that's closest to target heading 
        // (.i.e within +/-pi or target)
        float currentHeading = unwrapHeading(bearingToNextWaypoint, currentState.odometry.heading);

        headingPID.setpoint = bearingToNextWaypoint;
        desiredW = std::clamp(headingPID.calculate(currentHeading), -maxTurnVelocity, maxTurnVelocity);
        if (desiredV == 0.0){
            desiredW = 0;
        }
        printf("currentState.odometry.heading: %f, currentHeading: %f, headingPID.setpoint: %f, headingPID.calculate(currentHeading): %f \n", 
           currentState.odometry.heading,
           currentHeading,
           headingPID.setpoint,
           headingPID.calculate(currentHeading));
        float distanceToGo = distanceToWaypoint(targetWaypoint, currentState); 
        printf("Target Waypoint: %d, Distance To Go: %f, Nearest Waypoint: %d, bearing To Waypoint: %f, desiredw: %f ", targetWaypointIndex, distanceToGo, nearestWaypointIndex, bearingToNextWaypoint, desiredW);
        printf("X: %f, Y: %f, Velocity: %f, Heading: %f, turn rate: %f\n", 
           currentState.odometry.x,
           currentState.odometry.y,
           currentState.velocity.velocity,
           currentState.odometry.heading,
           currentState.velocity.angular_velocity);
    }
}

void WaypointNavigation::addWaypoint(const Waypoint& newWaypoint){
 // add a waypoint to the buffer in the last slot thats not NaN
    for (size_t i = 0; i < waypointBufferSize; i++) {
        if (isWaypointEmpty(waypointBuffer[i])) {
            waypointBuffer[i] = newWaypoint; // Add the new waypoint to the first empty slot found
            return; // Exit after adding the waypoint
        }
    }
    // If the function reaches this point, the buffer is full
    // TODO: extend buffer if it's full?
    // OR shuffle waypoints and indexes down one, if we're already past the first waypoint?
    printf("Waypoint buffer is full, unable to add new waypoint.\r\n");
}

// Function to check if a waypoint slot is considered empty (not populated)
bool WaypointNavigation::isWaypointEmpty(const Waypoint& waypoint) {
    // Check if any of the waypoint's properties are NaN
    return std::isnan(waypoint.position.x) || std::isnan(waypoint.position.y) ||
           std::isnan(waypoint.heading) || std::isnan(waypoint.speed);
}

uint8_t WaypointNavigation::nextWaypoint(const uint8_t currentWaypointIndex, const VehicleState& currentState){
    // find the next waypoint to navigate to. starts from the current (target) waypoint
    // and looks forward to find the closest one thats outside of the lookahead distance.
    // returns a waypoint index. Only looks/progresses forwards.
    uint8_t nextWaypointIndex = currentWaypointIndex;
    Waypoint targetWaypoint = waypointBuffer[currentWaypointIndex];
    while (distanceToWaypoint(targetWaypoint, currentState) < lookAhead ) { 
        // we don't enter/increment if the current target waypoint is already outside the lookahead
        nextWaypointIndex += 1;
        targetWaypoint = waypointBuffer[nextWaypointIndex];
    } 
    return nextWaypointIndex;
}

uint8_t WaypointNavigation::nearestWaypoint(const VehicleState& currentState){
    // returns the index of the waypoint nearest to the current position
    // searches through the full waypoint buffer, checking the distance of each
    uint8_t closestWaypointIndex = UINT8_MAX; //TODO check if it stays as UINT8_MAX
    float distanceToClosestWaypoint = std::numeric_limits<float>::max(); //initialise to max possible value

    for (uint8_t index = 0; index < waypointBufferSize; index++) { 
        Waypoint waypoint = waypointBuffer[index];
        float distance = distanceToWaypoint(waypoint, currentState);
        if (distance < distanceToClosestWaypoint){
            distanceToClosestWaypoint = distance;
            closestWaypointIndex = index;
        }
    } 
    return closestWaypointIndex;
}

void WaypointNavigation::clearWaypointBuffer(){ // clear all waypoints from buffer
    size_t i;

    // Initialize all waypoints in the buffer to NaN
    for (i=0; i < waypointBufferSize; i++) {
        waypointBuffer[i] = nanWaypoint;
    }
}

float WaypointNavigation::headingToWaypoint(const Waypoint& target, const VehicleState& currentState){
  // heading to a waypoint, relative to the current heading. i.e. a heading error
  // result is in radians, +/-pi

    return wrap_pi(bearingToWaypoint(target, currentState) - currentState.odometry.heading);
}

float WaypointNavigation::bearingToWaypoint(const Waypoint& target, const VehicleState& currentState){
 // bearing (heading) to a waypoint, relative to the "North" (Y axis) 
 // result is in radians
    float dx, dy, bearingToWaypoint;
    // bearing to waypoint is the compass heading from current location to the target waypoint
    dx = target.position.x - currentState.odometry.x;
    dy = target.position.y - currentState.odometry.y;
    if (dy != 0) {
        //x and Y flipped around in atan2 as we want angle from Yaxis,
        // not angle from X axis as the convention in maths
        //dx has a minus sign as we want clockwise positive, not anticlockwise as the convention for maths
        bearingToWaypoint = (float)atan2(-dx, dy); 
    } else {
        if (dx > 0) {
            bearingToWaypoint = M_PI / 2;
        } else {
            bearingToWaypoint = -M_PI / 2;
        }
    }
    return bearingToWaypoint;
}

float WaypointNavigation::distanceToWaypoint(const Waypoint& target, const VehicleState& currentState){
    // distance to a waypoint. assumes Waypoint and State both use the same units
    // returns the as-the-crow-flies distance between them

    //hypotenuse of dx, dy triangle gives distance, using h^2=x^2+y^2
    return sqrt(powf((target.position.x - currentState.odometry.x), 2)
                 + powf((target.position.y - currentState.odometry.y), 2));;
}

float WaypointNavigation::unwrapHeading(const float targetHeading, const float currentHeading){
    // finds "nearest" way to describe current heading compared to a target heading
    // to avoid a situation where current heading is 359 and target is 1, which
    // could give a heading error of 358. In that case, this function should return -1
    // alt example: current is -179, target is +179, function should return 181.
    // examples in degrees, function actually works in radians, so unwraps +/-2PI
    // works by finding the smallest way to describe the heading error,
    // then adding that error to the target to get back to the current heading

    float nearestHeading;
    float minHeadingError;
    minHeadingError = wrap_pi(targetHeading-currentHeading);
    nearestHeading = targetHeading - minHeadingError;
    return nearestHeading;
} 

WaypointNavigation::~WaypointNavigation() = default;

}