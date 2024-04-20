//
// Created by robbe on 02/12/2023.
//

#include "../include/stoker.h"

#include <cstdio>
#include <algorithm>

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
        float command_speed = speed + accel;

        motor.speed(pseudo_current_limit(current_motor_speed, command_speed));
        
        // print the motor position, current motors peed, speed, accel, and (speed + accel)
        //printf("millis(): %u  : %d, current motor speed: %f, setpoint: %f, accel: %f, duty: %f\n",
        //       millis(), motor_position_, current_motor_speed, speed, accel, motor.duty());
    }

    void Stoker::update(const VehicleState newState) {
        current_motor_speed = newState.driveTrainState.speeds[motor_position_];
    }


    float Stoker::pseudo_current_limit(float current_speed, float command_speed){
        // function attempts to limit tthe current the motor can draw, by constraining the commanded
        // speed to be close to the current speed. accepts current_speed and command_speed, in whatever 
        // units, but must be the same units as CONFIG::SPEED_SCALE_RADIANS_PER_SEC (which is assumed
        // to be the no load speed). aims to limit the current to +/-CONFIG::CURRENT_LIMIT Amps.

        float current_limited_speed = std::clamp(command_speed, current_speed - max_speed_change, current_speed + max_speed_change);

        //if (command_speed != current_limited_speed){
        //    printf("motor: %d's command limited from %f to %f, to reduce current draw\n",
        //                                 motor_position_, command_speed, current_limited_speed);
        //}
        return current_limited_speed;
    }

} // STOKER