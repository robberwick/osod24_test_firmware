//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_STOKER_H
#define OSOD_MOTOR_2040_STOKER_H

#include "drivers/motor/motor.hpp"

namespace STOKER {

    class Stoker {
    public:
        Stoker(const pin_pair &pins, Direction direction);
        void set_speed(float speed);

    private:
        motor::Motor motor;
    };

} // STOKER

#endif //OSOD_MOTOR_2040_STOKER_H
