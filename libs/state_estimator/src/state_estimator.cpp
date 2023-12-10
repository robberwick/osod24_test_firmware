//
// Created by robbe on 03/12/2023.
//
#include <cstdio>
#include "state_estimator.h"
#include "drivetrain_config.h"

namespace STATE_ESTIMATOR {
    StateEstimator *StateEstimator::instancePtr = nullptr;

    StateEstimator::StateEstimator() : encoders{
            .FRONT_LEFT =new Encoder(pio0, 0, motor2040::ENCODER_A),
            .FRONT_RIGHT =new Encoder(pio0, 1, motor2040::ENCODER_B),
            .REAR_LEFT = new Encoder(pio0, 2, motor2040::ENCODER_C),
            .REAR_RIGHT = new Encoder(pio0, 3, motor2040::ENCODER_D)
    }, timer(new repeating_timer_t) {
        encoders.FRONT_LEFT->init();
        encoders.FRONT_RIGHT->init();
        encoders.REAR_LEFT->init();
        encoders.REAR_RIGHT->init();

        instancePtr = this;
        setupTimer();
    }

    void StateEstimator::showValues() const {
        printf("FRONT_LEFT: %ld ", encoders.FRONT_LEFT->count());
        printf("FRONT_RIGHT: %ld ", encoders.FRONT_RIGHT->count());
        printf("REAR_LEFT: %ld ", encoders.REAR_LEFT->count());
        printf("REAR_RIGHT: %ld ", encoders.REAR_RIGHT->count());
        printf("\n");
    }

    void StateEstimator::publishState() const {
        showValues();
    }

    void StateEstimator::setupTimer() {
// Example configuration (adjust as needed)
        const uint32_t timerInterval = 50;  // Interval in milliseconds

        // Set up the repeating timer with the callback
        if (!add_repeating_timer_ms(timerInterval,
                                    reinterpret_cast<repeating_timer_callback_t>(&StateEstimator::timerCallback),
                                    nullptr, timer)) {
            // Handle error if timer creation fails
        }
    }

    void StateEstimator::timerCallback(repeating_timer_t *timer) {
        if (instancePtr != nullptr) {
            instancePtr->publishState();
        }
    }

    StateEstimator::~StateEstimator() {
        delete encoders.FRONT_LEFT;
        delete encoders.FRONT_RIGHT;
        delete encoders.REAR_LEFT;
        delete encoders.REAR_RIGHT;
        // Cancel the timer in the destructor
        cancel_repeating_timer(timer);

    }
}
// STATE_ESTIMATOR

