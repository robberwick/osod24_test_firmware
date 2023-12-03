//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_TANK_STEER_STRATEGY_H
#define OSOD_MOTOR_2040_TANK_STEER_STRATEGY_H
#include "mixer_strategy.h"


namespace MIXER {

    class TankSteerStrategy : public MixerStrategy {
    public:
        MotorSpeeds mix(float velocity, float angularVelocity) override;
    };
}


#endif //OSOD_MOTOR_2040_TANK_STEER_STRATEGY_H
