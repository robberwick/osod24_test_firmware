//
// Created by robbe on 02/12/2023.
//

#include "../include/stoker.h"

namespace STOKER {
    Stoker::Stoker(const pin_pair &pins, Direction direction = Direction::NORMAL_DIR) : motor(pins) {
        motor.init();
    }

    void Stoker::set_speed(float speed) {
        motor.speed(speed);

    }

    void Stoker::update(COMMON::DriveTrainState newState) {
        current_motor_speeds_ = newState.speeds;
    }

} // STOKER