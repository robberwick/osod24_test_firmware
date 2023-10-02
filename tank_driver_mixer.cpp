//
// Created by robbe on 01/10/2023.
//

#include "tank_driver_mixer.h"

struct MotorSpeed tank_steer_mix(float yaw, float throttle, float max_power) {
    float left = throttle + yaw;
    float right = throttle - yaw;
    float abs_left = left >= 0 ? left : -left;
    float abs_right = right >= 0 ? right : -right;
    float max_abs = abs_left > abs_right ? abs_left : abs_right;

    if (max_abs > 1.0) {
        left /= max_abs;
        right /= max_abs;
    }

    float scaled_left = (left * -max_power);
    float scaled_right = (right * -max_power);

    struct MotorSpeed result{};
    result.left = scaled_left;
    result.right = scaled_right;

    return result;
}