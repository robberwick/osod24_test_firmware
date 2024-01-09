//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_STOKER_H
#define OSOD_MOTOR_2040_STOKER_H

#include "interfaces.h"
#include "drivers/motor/motor.hpp"
#include "drivetrain_config.h"
#include "pid.hpp"

namespace STOKER {
    using namespace COMMON;
    class Stoker: public Observer {
    protected:
        ~Stoker() = default;

    public:
        Stoker(const pin_pair &pins, MOTOR_POSITION::MotorPosition position, Direction direction);
        void set_speed(float speed);

        void update(DriveTrainState newState) override;

    private:
        motor::Motor motor;
        float current_motor_speed = 0.0f;
        MOTOR_POSITION::MotorPosition motor_position_;

        /* UPDATE_RATE is a fixed number for now. 
        note the parameter doesn't drive updates, it's just used for scaling PID parameters 
        TODO: allow update rate to be settable on iniitalisation 
         and independent of calls to set_speed(), see issue #42 */
        float UPDATE_RATE = 0.02; //seconds
        PID vel_pid = PID(CONFIG::VEL_KP, CONFIG::VEL_KI, CONFIG::VEL_KD, UPDATE_RATE);
    };

} // STOKER

#endif //OSOD_MOTOR_2040_STOKER_H
