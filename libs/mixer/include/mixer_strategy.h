//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_MIXER_STRATEGY_H
#define OSOD_MOTOR_2040_MIXER_STRATEGY_H

namespace MIXER {

    // define a struct containing the motor speeds
    struct MotorSpeeds {
        float frontLeft;
        float frontRight;
        float rearLeft;
        float rearRight;
    };

    // define a struct containing the steering angles
    struct SteeringAngles {
        float left;
        float right;
    };

    struct MixerOutput {
        MotorSpeeds speeds;
        SteeringAngles angles;
    };

    class MixerStrategy {
    public:
        virtual MixerOutput mix(float velocity, float angularVelocity) = 0;
    };
}

#endif //OSOD_MOTOR_2040_MIXER_STRATEGY_H
