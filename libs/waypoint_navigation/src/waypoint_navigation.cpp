#include <cstdio>
#include <math.h>
#include <algorithm>
#include "types.h"
#include "drivetrain_config.h"
#include "waypoint_navigation.h"
#include "pico/stdlib.h"

namespace WAYPOINTS {

WaypointNavigation::WaypointNavigation(){}

void WaypointNavigation::navigate(const VehicleState& currentState) {
    if (mineDetected()) {
        //stop
        desiredW = desiredV = 0;
    } else {
        // go to next waypoint
        navigateToWaypoint(currentState);
    }
}

void WaypointNavigation::navigateToWaypoint(const VehicleState& currentState) {
    // updates desiredV and desiredW (speed and turn velocity)
    // based on the current position and the list of waypoints.
    // the velocity comes from the speed associated with the closest waypoint
    // the turn velocity comes from PID feedback on the heading to the next waypoint

    // find nearest waypoint and use it to set the speed
    nearestWaypointIndex = nearestWaypoint(currentState); //TODO: check if UINT8_MAX?
    Waypoint nearestWaypoint = CONFIG::waypointBuffer[nearestWaypointIndex];
    desiredV = nearestWaypoint.speed;

    // find target waypoint and use it to set the angular velocity
    targetWaypointIndex = nextWaypoint(targetWaypointIndex, currentState);
    targetWaypoint = CONFIG::waypointBuffer[targetWaypointIndex];
    float bearingToNextWaypoint = bearingToWaypoint(targetWaypoint, currentState);

    // reframe current heading to be described in a way that's closest to target heading 
    // (.i.e within +/-pi or target)
    float currentHeading = unwrapHeading(bearingToNextWaypoint, currentState.odometry.heading);

    headingPID.setpoint = bearingToNextWaypoint;
    //scale the response by the speed, so that the steering correction angle is consistent as run speeds varies
    desiredW = std::clamp(-desiredV * headingPID.calculate(currentHeading),
                                -maxTurnVelocity, maxTurnVelocity);
    float distanceToGo = distanceToWaypoint(targetWaypoint, currentState); 
    printf("Target Waypoint: %d, Distance To Go: %f, Nearest Waypoint: %d, bearing To Waypoint: %f, desiredV: %f ", targetWaypointIndex, distanceToGo, nearestWaypointIndex, bearingToNextWaypoint, desiredV);
    printf("X: %f, Y: %f, Velocity: %f, Heading: %f, turn rate: %f\n", 
        currentState.odometry.x,
        currentState.odometry.y,
        currentState.velocity.velocity,
        currentState.odometry.heading,
        currentState.velocity.angular_velocity);
}


uint8_t WaypointNavigation::nextWaypoint(const uint8_t currentWaypointIndex, const VehicleState& currentState){
    // find the next waypoint to navigate to. starts from the current (target) waypoint
    // and looks forward to find the closest one thats outside of the lookahead distance.
    // returns a waypoint index. Only looks/progresses forwards.
    uint8_t nextWaypointIndex = currentWaypointIndex;
    Waypoint targetWaypoint = CONFIG::waypointBuffer[currentWaypointIndex];
    while (distanceToWaypoint(targetWaypoint, currentState) < lookAhead ) { 

        // we don't enter/increment if the current target waypoint is already outside the lookahead
        nextWaypointIndex += 1;
        if (nextWaypointIndex == CONFIG::waypointCount){
            //if we've reached the end of the list of waypoints, loop back to the first waypoint
            nextWaypointIndex = 0;
        }
        targetWaypoint = CONFIG::waypointBuffer[nextWaypointIndex];
    } 
    return nextWaypointIndex;
}

uint8_t WaypointNavigation::nearestWaypoint(const VehicleState& currentState){
    // returns the index of the waypoint nearest to the current position
    // searches only through the waypoint buffer from the current(previous)
    // closest waypoint up to the target waypoint, checking the distance of each
    uint8_t closestWaypointIndex = nearestWaypointIndex; 
    float distanceToClosestWaypoint = std::numeric_limits<float>::max(); //initialise to max possible value
    uint8_t lookaheadLimit = targetWaypointIndex;
    if (nearestWaypointIndex > targetWaypointIndex) {
        //if we've looping around and the targetwaypoint is at the start of the buffer,
        // but the current nearest is towards the end then set the limit proportionally ahead
        uint8_t lookaheadLimit = targetWaypointIndex + CONFIG::waypointCount;
    }
    
    for (uint8_t index = nearestWaypointIndex; index <= lookaheadLimit; index++) {
        // Wrap the index around if it exceeds the buffer size
        uint8_t wrappedIndex = index % CONFIG::waypointCount;

        Waypoint waypoint = CONFIG::waypointBuffer[wrappedIndex];
        float distance = distanceToWaypoint(waypoint, currentState);

        if (distance < distanceToClosestWaypoint){
            distanceToClosestWaypoint = distance;
            closestWaypointIndex = wrappedIndex;
        }
    }
    return closestWaypointIndex;
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
    nearestHeading = targetHeading+ minHeadingError;
    return nearestHeading;
} 

bool WaypointNavigation::mineDetected(){
    using namespace CONFIG;
    for (int pin = 0; pin < numMineSensors; pin++) {
        if (gpio_get(mineSensorPins[pin])) {  // Check if a mine has been detected
            return true;
        }
    }
    return false;  // Return false if no mines detected
}

WaypointNavigation::~WaypointNavigation() = default;

}