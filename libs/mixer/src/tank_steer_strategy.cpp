//
// Created by markmellors on 03/12/2023.
//

#include "tank_steer_strategy.h"

using namespace MIXER;

MotorSpeeds TankSteerStrategy::mix(float velocity, float angularVelocity) {
    float left = velocity - angularVelocity * track_width / 2;
    float right = -velocity - angularVelocity * track_width / 2;
    return {left, right, left, right};
}
