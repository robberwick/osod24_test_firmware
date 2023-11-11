//
// Created by markm on 11/11/2023.
//
// code for a four wheeled, independent four wheel drive, two wheel steering platform

#include "ackermann_mixer.h"

AckermannMixer::AckermannMixer(float track, float base, float angle) : wheelTrack(track), wheelBase(base), maxSteeringAngle(angle) {} 

struct MotorSpeed4wd AckermannMixer::ackermann_steer_mix(float yaw, float throttle){
    // function takes desired forward speed in mm/s and turn rate in radians/sec
    // and outputs individual wheel speeds in mm/sec and turn angle of steerable wheels
    
    struct MotorSpeed4wd result{};
    result.frontleft = 0;
    result.frontright = 0;
    result.rearleft = 0;
    result.rearright = 0;
    return result;
}