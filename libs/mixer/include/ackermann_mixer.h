//
// Created by markm on 11/11/2023.
//

#ifndef OSOD_MOTOR_2040_ACKERMANN_MIXER_H
#define OSOD_MOTOR_2040_ACKERMANN_MIXER_H

#include "drivetrain_config.h"
#include "mixer_strategy.h"

namespace MIXER {
    
    class AckermannMixer : public MixerStrategy {
    private:
        float wheelTrack;         // Distance between the left and right wheels (m)
        float wheelBase;          // Distance between the front and rear wheels (m)
        float maxSteeringAngle;   // Max angle from straight ahead that the steerable wheels can pivot (radians)
        float turnRadius;         // calculated turn radius from inputs, m to centreline

    public:
        AckermannMixer(float track = CONFIG::WHEEL_TRACK,
                    float base = CONFIG::WHEEL_BASE,
                    float angle = CONFIG::MAX_STEERING_ANGLE); // constructor

        float getTurnRadius() const; //getter for turn radius
        void setMaxSteeringAngle(float angle); //getter for turn radius
        AckermannOutput mix(float velocity, float angularVelocity) override; //mixing function

    };
} // namespace MIXER
#endif //OSOD_MOTOR_2040_ACKERMANN_MIXER_H
