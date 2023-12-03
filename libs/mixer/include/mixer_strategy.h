//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_MIXER_STRATEGY_H
#define OSOD_MOTOR_2040_MIXER_STRATEGY_H

namespace MIXER {

    // define a struct containing the motor speeds
    struct MotorSpeeds {
        float FRONT_LEFT;
        float FRONT_RIGHT;
        float REAR_LEFT;
        float REAR_RIGHT;
    };

    class MixerStrategy {
    public:
        virtual MotorSpeeds mix(float velocity, float angularVelocity, float speed_factor) = 0;
    };
}

#endif //OSOD_MOTOR_2040_MIXER_STRATEGY_H
