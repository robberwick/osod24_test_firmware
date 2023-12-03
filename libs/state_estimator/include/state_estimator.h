//
// Created by robbe on 03/12/2023.
//

#ifndef OSOD_MOTOR_2040_STATE_ESTIMATOR_H
#define OSOD_MOTOR_2040_STATE_ESTIMATOR_H

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "motor2040.hpp"

using namespace motor;
using namespace encoder;
struct Encoders {
    Encoder *FRONT_LEFT;
    Encoder *FRONT_RIGHT;
    Encoder *REAR_LEFT;
    Encoder *REAR_RIGHT;
};

namespace STATE_ESTIMATOR {

    class StateEstimator {
    public:
        explicit StateEstimator();
        ~StateEstimator();  // Destructor to cancel the timer
        void showValues() const;
        void publishState() const;

    private:
        Encoders encoders;
        static StateEstimator *instancePtr;
        repeating_timer_t* timer;

        static void timerCallback(repeating_timer_t* timer);

        void setupTimer();

    };

} // STATE_ESTIMATOR

#endif //OSOD_MOTOR_2040_STATE_ESTIMATOR_H
