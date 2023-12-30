//
// Created by markm on 11/11/2023.
//
// code for a four wheeled, independent four wheel drive, two wheel steering platform

#include "ackermann_mixer.h"
#include "mixer_strategy.h"
#include <cmath>
#include <limits>
#include <cstdio>

/**
 * @brief Returns the sign of a value
 * @tparam T The type of the value
 * @param value The value to check
 * @return 1 if the value is positive, -1 if the value is negative
 */
template<typename T>
int sign(T value) {
    return std::signbit(value) ? -1 : 1;
}

namespace MIXER {
AckermannMixer::AckermannMixer(float track, float base, float angle) : wheelTrack(track), wheelBase(base), maxSteeringAngle(angle) {} 

AckermannOutput AckermannMixer::mix(float velocity, float angularVelocity){
    // function takes desired forward speed ("throttle") in mm/s and turn rate in radians/sec
    // and outputs individual wheel speeds in mm/sec and turn angle of steerable wheels in radians

    printf("velocity: %.2f ", velocity);
    printf(", yaw: %.2f ", angularVelocity);

    if (std::fabs(angularVelocity) > 0) {
        turnRadius = velocity / angularVelocity;
    } else {
        // Handle the case when yaw rate is close to zero to avoid division by zero
        turnRadius = std::numeric_limits<float>::infinity();
    }
    printf(", turnRadius: %.2f ", turnRadius);
    AckermannOutput result;
    if (std::isinf(turnRadius)) {
        // if the turn radius is infinite then we're not turning, so all wheels travel 
        // at the desired forwards speed and the steerable wheels point forwards
        result.frontLeftSpeed = velocity;
        result.frontRightSpeed = -velocity;
        result.rearLeftSpeed = velocity;
        result.rearRightSpeed = -velocity;
        result.frontLeftAngle = 0;
        result.frontRightAngle = 0;
    } else {
        if (velocity == 0) {
        // if we're only turning, the speeds are symmetrical and just depends on the turn rate
        // and the angle depends on the wheelbase and track
            result.frontLeftSpeed = -angularVelocity * std::sqrt((wheelTrack / 2) * (wheelTrack / 2) + wheelBase * wheelBase);
            result.frontRightSpeed = result.frontLeftSpeed;
            result.rearLeftSpeed = -angularVelocity * wheelTrack / 2;
            result.rearRightSpeed = result.rearLeftSpeed;
            result.frontLeftAngle = std::atan2(wheelBase,  turnRadius - wheelTrack / 2);
            result.frontRightAngle = std::atan2(wheelBase, turnRadius + wheelTrack / 2);
        } else {
            result.frontLeftSpeed = angularVelocity * std::sqrt((turnRadius - wheelTrack / 2) * (turnRadius - wheelTrack / 2) + wheelBase * wheelBase);
            result.frontLeftSpeed = result.frontLeftSpeed *  sign(turnRadius - wheelTrack / 2);
            result.frontRightSpeed = angularVelocity * std::sqrt((turnRadius + wheelTrack / 2) * (turnRadius + wheelTrack / 2) + wheelBase * wheelBase);
            result.frontRightSpeed = -result.frontRightSpeed * sign(turnRadius + wheelTrack / 2);
            result.rearLeftSpeed = velocity * (turnRadius - wheelTrack / 2) / turnRadius;
            result.rearRightSpeed = -velocity * (turnRadius + wheelTrack / 2) / turnRadius;          
            float tempFrontLeftAngle = -std::atan(wheelBase/(turnRadius - wheelTrack / 2));
            float tempFrontRightAngle = std::atan(wheelBase/(turnRadius + wheelTrack / 2));
            // constrain steering angles
            result.frontLeftAngle = std::fmin(std::fmax(tempFrontLeftAngle, -maxSteeringAngle), maxSteeringAngle);
            result.frontRightAngle = std::fmin(std::fmax(tempFrontRightAngle, -maxSteeringAngle), maxSteeringAngle);

            // modify speeds to correct for limited steering
            result.frontLeftSpeed = result.frontLeftSpeed * std::cos(tempFrontLeftAngle - result.frontLeftAngle);
            result.frontRightSpeed = result.frontRightSpeed * std::cos(tempFrontRightAngle - result.frontRightAngle);
        }

    }
    
    printf(", FL Speed: %.2f ", result.frontLeftSpeed);
    printf(", FR Speed: %.2f ", result.frontRightSpeed);
    printf(", RL Speed: %.2f ", result.rearLeftSpeed);
    printf(", RR Speed: %.2f ", result.rearRightSpeed);
    printf(", FL Angle: %.2f ", result.frontLeftAngle);
    printf("FR Angle: %.2f\n", result.frontRightAngle);
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

} // namespace MIXER