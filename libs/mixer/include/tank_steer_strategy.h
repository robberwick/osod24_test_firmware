//
// Created by markmellors on 03/12/2023.
//

#ifndef OSOD_MOTOR_2040_TANK_STEER_STRATEGY_H
#define OSOD_MOTOR_2040_TANK_STEER_STRATEGY_H
#include "mixer_strategy.h"


namespace MIXER {

    class TankSteerStrategy : public MixerStrategy {
        // This class implements a steering strategy for a tank-like robot.
        // It calculates the motor speeds based on the given linear and angular velocities.
    public:
        // Constructor to set the track width and wheel base
        // @param trackWidth The distance between the centers of the left and right wheels (in meters).
        // @param wheelBase The distance between the front and rear axles (in meters).
        TankSteerStrategy(float trackWidth, float wheelBase)
            : track_width(trackWidth), wheel_base(wheelBase) {}
        
        // Calculate motor speeds based on provided linear and angular velocities.
        // @param velocity The desired forward velocity of the robot (in meters per second).
        // @param angularVelocity The desired rotational velocity of the robot (in radians per second).
        // @return MotorSpeeds The speeds for each motor (in meters per second).
        MotorSpeeds mix(float velocity, float angularVelocity) override;

    private:
        float track_width;  // Track width in meters
        float wheel_base;   // Wheel base in meters
    };
}


#endif //OSOD_MOTOR_2040_TANK_STEER_STRATEGY_H
