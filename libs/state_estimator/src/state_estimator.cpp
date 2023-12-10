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

        // Initialize the State struct member variables
        estimatedState.x = 0.0f;
        estimatedState.xdot = 0.0f;
        estimatedState.y = 0.0f;
        estimatedState.ydot = 0.0f;
        estimatedState.velocity = 0.0f;
        estimatedState.heading = 0.0f;
        estimatedState.angularVelocity = 0.0f;
        estimatedState.FL_wheel_speed = 0.0f;
        estimatedState.FR_wheel_speed = 0.0f;
        estimatedState.RL_wheel_speed = 0.0f;
        estimatedState.RR_wheel_speed = 0.0f;
        
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

    void StateEstimator::estimateState() {
        // calculate position deltas
        float distance_travelled;
        float heading_change;
        float left_travel;
        float right_travel;
        //get average wheel rotation for left side
        left_travel = ((encoders.FRONT_LEFT->radians()) + (encoders.REAR_LEFT->radians())) / 2;
        // convert wheel rotation to distance travelled in meters
        left_travel = left_travel * CONFIG::WHEEL_DIAMETER / 2;
        right_travel = ((encoders.FRONT_RIGHT->radians()) + (encoders.REAR_RIGHT->radians())) / 2;
        right_travel = right_travel * CONFIG::WHEEL_DIAMETER / 2;

        distance_travelled = (left_travel - right_travel) / 2;
        heading_change = (left_travel + right_travel) / CONFIG::WHEEL_TRACK;

        //calculate new position and orientation
        //calc a temp heading halfway between old heading and new
        //assumed to be representative of heading during distance_travelled
        float tempHeading;
        tempHeading = estimatedState.heading + heading_change / 2;
        estimatedState.x = estimatedState.x + distance_travelled * sin(tempHeading);
        estimatedState.y = estimatedState.y + distance_travelled * cos(tempHeading);

        //now actually update heading
        estimatedState.heading = estimatedState.heading + heading_change;

        //calculate speeds
        float left_speed;
        float right_speed;
        
        //get wheel speeds
        estimatedState.FL_wheel_speed = encoders.FRONT_LEFT->capture().radians_per_second();
        estimatedState.FR_wheel_speed = encoders.FRONT_RIGHT->capture().radians_per_second();
        estimatedState.RL_wheel_speed = encoders.REAR_LEFT->capture().radians_per_second();
        estimatedState.RR_wheel_speed = encoders.REAR_RIGHT->capture().radians_per_second();
        left_speed = (estimatedState.FL_wheel_speed + estimatedState.RL_wheel_speed) / 2;
        
        // convert wheel rotation to distance travelled in meters
        left_speed = left_speed * CONFIG::WHEEL_DIAMETER / 2;

        //repeat for right side
        right_speed = (estimatedState.FR_wheel_speed + estimatedState.RR_wheel_speed) / 2;
        right_speed = right_speed * CONFIG::WHEEL_DIAMETER / 2;

        //calc all velocities
        estimatedState.velocity = (left_speed - right_speed) / 2;
        estimatedState.xdot = estimatedState.velocity * sin(estimatedState.heading);
        estimatedState.ydot = estimatedState.velocity * cos(estimatedState.heading);
        estimatedState.angularVelocity= (left_speed + right_speed) / CONFIG::WHEEL_TRACK;

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

