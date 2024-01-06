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
        Stoker(const pin_pair &pins, Direction direction);
        void set_speed(float speed);

        void update(COMMON::DriveTrainState newState) override;

    private:
        motor::Motor motor;
        COMMON::MotorSpeeds current_motor_speeds_;
    };

} // STOKER

#endif //OSOD_MOTOR_2040_STOKER_H
