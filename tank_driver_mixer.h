//
// Created by robbe on 01/10/2023.
//

#ifndef OSOD_MOTOR_2040_TANK_DRIVER_MIXER_H
#define OSOD_MOTOR_2040_TANK_DRIVER_MIXER_H


struct MotorSpeed {
    float left;
    float right;
};

struct MotorSpeed tank_steer_mix(float yaw, float throttle, float max_power);

#endif //OSOD_MOTOR_2040_TANK_DRIVER_MIXER_H
