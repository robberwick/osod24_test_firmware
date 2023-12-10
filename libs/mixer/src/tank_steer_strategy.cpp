//
// Created by markmellors on 03/12/2023.
//

#include "tank_steer_strategy.h"

using namespace MIXER;

MotorSpeeds TankSteerStrategy::mix(float velocity, float angularVelocity) {
    float left = velocity + angularVelocity;
    float right = velocity - angularVelocity;
    float abs_left = left >= 0 ? left : -left;
    float abs_right = right >= 0 ? right : -right;
    float max_abs = abs_left > abs_right ? abs_left : abs_right;

    if (max_abs > 1.0) {
        left /= max_abs;
        right /= max_abs;
    }
    const float speed_factor = 1.0f;
    float scaled_left = (left * speed_factor);
    float scaled_right = (right * speed_factor) * -1;
    return {scaled_left, scaled_right, scaled_left, scaled_right};
}
