//
// Created by robbe on 03/12/2023.
//
#include <cstdio>
#include "state_estimator.h"
#include "drivetrain_config.h"

namespace STATE_ESTIMATOR {
    StateEstimator *StateEstimator::instancePtr = nullptr;

    StateEstimator::StateEstimator() : encoders{
            .FRONT_LEFT =new Encoder(pio0, 0, motor2040::ENCODER_A, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            .FRONT_RIGHT =new Encoder(pio0, 1, motor2040::ENCODER_B, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            .REAR_LEFT = new Encoder(pio0, 2, motor2040::ENCODER_C, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            .REAR_RIGHT = new Encoder(pio0, 3, motor2040::ENCODER_D, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV)
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
        estimatedState.driveTrainState.speeds.frontLeft = 0.0f;
        estimatedState.driveTrainState.speeds.frontRight = 0.0f;
        estimatedState.driveTrainState.speeds.frontRight = 0.0f;
        estimatedState.driveTrainState.speeds.rearRight = 0.0f;
        estimatedState.driveTrainState.angles.left = 0.0f;
        estimatedState.driveTrainState.angles.right = 0.0f;
        
        instancePtr = this;
        setupTimer();
    }

    void StateEstimator::showValues() const {
        printf("FRONT_LEFT: %ld ", encoders.FRONT_LEFT->count());
        printf("FRONT_RIGHT: %ld ", encoders.FRONT_RIGHT->count());
        printf("REAR_LEFT: %ld ", encoders.REAR_LEFT->count());
        printf("REAR_RIGHT: %ld ", encoders.REAR_RIGHT->count());
        printf("\n");
        printf("X: %f, Y: %f, Velocity: %f, Heading: %f, turn rate: %f\n", 
           estimatedState.x, 
           estimatedState.y, 
           estimatedState.velocity, 
           estimatedState.heading,
           estimatedState.angularVelocity);
    }

    void StateEstimator::publishState() const {
        showValues();
    }

    void StateEstimator::estimateState() {
        
        //get current encoder state
        const auto captureFL = encoders.FRONT_LEFT->capture();
        const auto captureFR = encoders.FRONT_RIGHT->capture();
        const auto captureRL = encoders.REAR_LEFT->capture();
        const auto captureRR = encoders.REAR_RIGHT->capture();
        
        // calculate position deltas

        // Calculate average wheel rotation delta for left and right sides
        float left_travel = (captureFL.radians_delta() + captureRL.radians_delta()) / 2;
        float right_travel = (captureFR.radians_delta() + captureRR.radians_delta()) / 2;
        
        // convert wheel rotation to distance travelled in meters
        left_travel = left_travel * CONFIG::WHEEL_DIAMETER / 2;
        right_travel = right_travel * CONFIG::WHEEL_DIAMETER / 2;

        const float distance_travelled = (left_travel - right_travel) / 2;
        const float heading_change = (left_travel + right_travel) / CONFIG::WHEEL_TRACK;

        //calculate new position and orientation
        //calc a temp heading halfway between old heading and new
        //assumed to be representative of heading during distance_travelled
        const float tempHeading = estimatedState.heading + heading_change / 2;
        estimatedState.x = estimatedState.x + distance_travelled * sin(tempHeading);
        estimatedState.y = estimatedState.y + distance_travelled * cos(tempHeading);

        //now actually update heading
        estimatedState.heading = estimatedState.heading + heading_change;
        
        //constrain heading to +/-pi 
        if (estimatedState.heading > M_PI) {
            estimatedState.heading -= 2 * M_PI;
        }
        if (estimatedState.heading < -M_PI) {
            estimatedState.heading += 2 * M_PI;
        }

        //calculate speeds

        //get wheel speeds
        estimatedState.driveTrainState.speeds.frontLeft = captureFL.radians_per_second();
        estimatedState.driveTrainState.speeds.frontRight = captureFR.radians_per_second();
        estimatedState.driveTrainState.speeds.rearLeft = captureRL.radians_per_second();
        estimatedState.driveTrainState.speeds.rearRight = captureRR.radians_per_second();

        // average wheel speed in radians per sed
        float left_speed = (estimatedState.driveTrainState.speeds.frontLeft + estimatedState.driveTrainState.speeds.
                            rearLeft) / 2;
        
        // convert average wheel rotation speed to linear speed
        left_speed = left_speed * CONFIG::WHEEL_DIAMETER / 2;

        //repeat for right side
        float right_speed = (estimatedState.driveTrainState.speeds.frontRight + estimatedState.driveTrainState.speeds.
                             rearRight) / 2;
        right_speed = right_speed * CONFIG::WHEEL_DIAMETER / 2;

        //calc all velocities
        estimatedState.velocity = (left_speed - right_speed) / 2;
        estimatedState.xdot = estimatedState.velocity * sin(estimatedState.heading);
        estimatedState.ydot = estimatedState.velocity * cos(estimatedState.heading);
        estimatedState.angularVelocity= (left_speed + right_speed) / CONFIG::WHEEL_TRACK;

    }

    void StateEstimator::setupTimer() const {
        // Example configuration (adjust as needed)
        constexpr uint32_t timerInterval = 50;  // Interval in milliseconds

        // Set up the repeating timer with the callback
        if (!add_repeating_timer_ms(timerInterval,
                                    reinterpret_cast<repeating_timer_callback_t>(&StateEstimator::timerCallback),
                                    nullptr, timer)) {
            // Handle error if timer creation fails
        }
    }

    void StateEstimator::timerCallback(repeating_timer_t *timer) {
        if (instancePtr != nullptr) {
            instancePtr->estimateState();
            instancePtr->publishState();
        }
    }

    void StateEstimator::updateCurrentDriveTrainState(const COMMON::DriveTrainState& newDriveTrainState) {
        currentDriveTrainState = newDriveTrainState;
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

