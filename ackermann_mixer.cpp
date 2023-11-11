//
// Created by markm on 11/11/2023.
//
// code for a four wheeled, independent four wheel drive, two wheel steering platform

#include "ackermann_mixer.h"
#include <cmath>
#include <limits>

AckermannMixer::AckermannMixer(float track, float base, float angle) : wheelTrack(track), wheelBase(base), maxSteeringAngle(angle) {} 

struct AckermannOutput AckermannMixer::ackermann_steer_mix(float yaw, float throttle){
    // function takes desired forward speed ("throttle") in mm/s and turn rate in radians/sec
    // and outputs individual wheel speeds in mm/sec and turn angle of steerable wheels in radians

    if (std::fabs(yaw) > 0) {
        turnRadius = throttle / yaw;
    } else {
        // Handle the case when yaw rate is close to zero to avoid division by zero
        turnRadius = std::numeric_limits<float>::infinity();
    }

    struct AckermannOutput result;

    if (turnRadius == 0) {
        // if we're not turning, all wheels travel at the desired forwards speed
        // and the steerable wheels point forwards
        result.frontLeftSpeed = throttle;
        result.frontRightSpeed = throttle;
        result.rearLeftSpeed = throttle;
        result.rearRightSpeed = throttle;
        result.frontLeftAngle = 0
        result.frontRightAngle = 0
    } else {
        if (throttle == 0) {
        // if we're only turning, the speeds are symetrical and just depends on the turn rate
        // and the angle depends on the wheel base and track
            result.frontLeftSpeed = turnRadius * std::sqrt((wheelTrack / 2) * (wheelTrack / 2) + wheelBase * wheelBase);
            result.frontRightSpeed = -result.frontLeftSpeed;
            result.rearLeftSpeed = turnRadius * wheelTrack / 2;
            result.rearRightSpeed = -result.rearLeftSpeed;
            result.frontLeftAngle = std::atan2(wheelBase,  turnRadius - wheelTrack / 2);
            result.frontrightAngle = std::atan2(wheelBase, turnRadius + wheelTrack / 2);
        } else {
            result.frontLeftSpeed = yaw * std::sqrt((turnRadius - wheelTrack / 2) * (turnRadius - wheelTrack / 2) + wheelBase * wheelBase);
            result.frontLeftSpeed = std::copysign(result.frontLeftSpeed, (turnRadius - wheelTrack / 2));
            result.frontRightSpeed = yaw * std::sqrt((turnRadius + wheelTrack / 2) * (turnRadius + wheelTrack / 2) + wheelBase * wheelBase);
            result.frontRightSpeed = std::copysign(result.frontRightSpeed, (turnRadius + wheelTrack / 2));
            result.rearLeftSpeed = throttle * (turnRadius - wheelTrack / 2) / turnRadius;
            result.rearRightSpeed = -throttle * (turnRadius + wheelTrack / 2) / turnRadius;          
            float tempFrontLeftAngle = std::atan2(wheelBase,  turnRadius - wheelTrack / 2);
            float tempFrontRightAngle = std::atan2(wheelBase, turnRadius + wheelTrack / 2);
            
            // constrain steering angles
            result.frontLeftAngle = std::fmin(std::fmax(tempFrontLeftAngle, -maxSteeringAngle), maxSteeringAngle);
            result.frontRightAngle = std::fmin(std::fmax(tempFrontRightAngle, -maxSteeringAngle), maxSteeringAngle);

            // modify speeds to correct for limited steering
            result.frontLeftSpeed = result.frontLeftSpeed * std::cos(tempFrontLeftAngle - result.frontLeftAngle);
            result.frontRightSpeed = result.frontRightSpeed * std::cos(tempFrontRightAngle - result.frontRightAngle);
        }
    }
    return result;
}

float AckermannMixer::getTurnRadius() const {
    //returns the current radius of turn (to the robots centreline) in mm
    return turnRadius;
}

void AckermannMixer::setMaxSteeringAngle(float angle){
    // sets the maximum steering angle fo the steerable wheels, in radians
    maxSteeringAngle = angle;
}