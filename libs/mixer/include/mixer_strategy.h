//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_MIXER_STRATEGY_H
#define OSOD_MOTOR_2040_MIXER_STRATEGY_H

namespace MIXER {

    // define a struct containing the motor speeds
    struct AckermannOutput {
        float frontLeftSpeed;
        float frontRightSpeed;
        float rearLeftSpeed;
        float rearRightSpeed;
        float frontLeftAngle;
        float frontRightAngle;
    };

    class MixerStrategy {
    public:
        virtual AckermannOutput mix(float velocity, float angularVelocity) = 0;
    };
}

#endif //OSOD_MOTOR_2040_MIXER_STRATEGY_H
