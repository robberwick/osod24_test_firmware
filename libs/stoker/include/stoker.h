//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_STOKER_H
#define OSOD_MOTOR_2040_STOKER_H

#include "interfaces.h"
#include "drivers/motor/motor.hpp"

namespace STOKER {

    class Stoker: public COMMON::Observer {
    protected:
        ~Stoker() = default;

    public:
        Stoker(const pin_pair &pins, COMMON::MOTOR_POSITION::MotorPosition position, Direction direction);
        void set_speed(float speed);

        void update(COMMON::DriveTrainState newState) override;

    private:
        motor::Motor motor;
        float current_motor_speed = 0.0f;
        COMMON::MOTOR_POSITION::MotorPosition motor_position_;
    };

} // STOKER

#endif //OSOD_MOTOR_2040_STOKER_H
