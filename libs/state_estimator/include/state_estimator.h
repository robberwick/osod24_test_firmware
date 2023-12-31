//
// Created by robbe on 03/12/2023.
//

#ifndef OSOD_MOTOR_2040_STATE_ESTIMATOR_H
#define OSOD_MOTOR_2040_STATE_ESTIMATOR_H

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "motor2040.hpp"
#include "drivetrain_config.h"
#include "bno080.h"

using namespace motor;
using namespace encoder;
struct Encoders {
    Encoder *FRONT_LEFT;
    Encoder *FRONT_RIGHT;
    Encoder *REAR_LEFT;
    Encoder *REAR_RIGHT;
};

namespace STATE_ESTIMATOR {

    // define a State struct containing the state parameters that can be requested or tracked
    struct State {
        float x;
        float xdot;
        float y;
        float ydot;
        float velocity;
        float heading;
        float angularVelocity;
        float FL_wheel_speed;
        float FR_wheel_speed;
        float RL_wheel_speed;
        float RR_wheel_speed;
    };
    class StateEstimator {
    public:
        explicit StateEstimator(i2c_inst_t* port);

        ~StateEstimator();  // Destructor to cancel the timer
        void showValues() const;
        void estimateState();
        void publishState() const;

    private:
        Encoders encoders;
        BNO08x IMU;
        static StateEstimator *instancePtr;
        repeating_timer_t *timer;
        State estimatedState;
        State previousState;
        static void timerCallback(repeating_timer_t *timer);

        void setupTimer();

    };

} // STATE_ESTIMATOR

#endif //OSOD_MOTOR_2040_STATE_ESTIMATOR_H
