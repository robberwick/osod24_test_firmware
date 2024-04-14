//
// Created by robbe on 02/12/2023.
//

#include "../include/stoker.h"

#include <cstdio>

namespace STOKER {
    Stoker::Stoker(const pin_pair& pins, const MOTOR_POSITION::MotorPosition position,
                   Direction direction = Direction::NORMAL_DIR) : motor(pins, direction,
                                                                        CONFIG::SPEED_SCALE_RADIANS_PER_SEC),
                                                                  motor_position_(position) {
        motor.init();
    }

    void Stoker::set_speed(const float speed) {
        vel_pid.setpoint = speed;
        float accel = vel_pid.calculate(current_motor_speed);
        motor.speed(speed + accel);
        // print the motor position, current motors peed, speed, accel, and (speed + accel)
        printf("millis(): %u  : %d, current motor speed: %f, setpoint: %f, accel: %f, speed + accel: %f\n",
               millis(), motor_position_, current_motor_speed, speed, accel, speed + accel);
    }

    void Stoker::update(const VehicleState newState) {
        current_motor_speed = newState.driveTrainState.speeds[motor_position_];
    }

} // STOKER