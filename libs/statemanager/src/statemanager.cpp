//
// Created by robbe on 02/12/2023.
//

#include <cstdio>
#include "state_estimator.h"
#include "statemanager.h"

#include <libraries/pico_synth/pico_synth.hpp>
#include <libraries/servo2040/servo2040.hpp>
#include "motor2040.hpp"
#include "servo.hpp"

namespace STATEMANAGER {
    void StateManager::initialiseServo(servo::Servo*& servo, const uint pin, const float minPulse, const float midPulse, const float maxPulse, const float minValue = -0.7854, const float midValue = 0, const float maxValue = 0.7854) {
        servo = new servo::Servo(pin);
        servo->init();
        servo->calibration().apply_three_pairs(minPulse, midPulse, maxPulse, minValue, midValue, maxValue);
        servo->enable();
        servo->to_mid();
    }

    StateManager::StateManager(MIXER::MixerStrategy *mixerStrategy, STATE_ESTIMATOR::StateEstimator *stateEstimator) : mixerStrategy(mixerStrategy), stateEstimator(stateEstimator) {
        printf("creating State manager\n");
        // set up the stokers
        stokers[MOTOR_POSITION::FRONT_LEFT] = new STOKER::Stoker(motor::motor2040::MOTOR_A, MOTOR_POSITION::FRONT_LEFT, Direction::REVERSED_DIR);
        stokers[MOTOR_POSITION::FRONT_RIGHT] = new STOKER::Stoker(motor::motor2040::MOTOR_B, MOTOR_POSITION::FRONT_RIGHT, Direction::REVERSED_DIR);
        stokers[MOTOR_POSITION::REAR_LEFT] = new STOKER::Stoker(motor::motor2040::MOTOR_C, MOTOR_POSITION::REAR_LEFT, Direction::REVERSED_DIR);
        stokers[MOTOR_POSITION::REAR_RIGHT] = new STOKER::Stoker(motor::motor2040::MOTOR_D, MOTOR_POSITION::REAR_RIGHT, Direction::REVERSED_DIR);

        stateEstimator->addObserver(stokers[MOTOR_POSITION::FRONT_LEFT]);
        stateEstimator->addObserver(stokers[MOTOR_POSITION::FRONT_RIGHT]);
        stateEstimator->addObserver(stokers[MOTOR_POSITION::REAR_LEFT]);
        stateEstimator->addObserver(stokers[MOTOR_POSITION::REAR_RIGHT]);

        // set up the servos
        // left - ADC2 / PWM 6 - Pin 28
        initialiseServo(steering_servos.left, motor2040::RX_ECHO, 1221, 1750, 2200);
        // right - TX_TRIG / PWM 0 - Pin 16
        initialiseServo(steering_servos.right, motor2040::TX_TRIG, 1900, 1400, 1000);
        printf("State manager created\n");
    }

    void StateManager::requestState(const COMMON::VehicleState& requestedState) {
        //printf("Requested state...\n");
        //printf("Velocity: %f ", requestedState.velocity);
        //printf("Angular velocity: %f ", requestedState.angularVelocity);
        //printf("\n");
        const DriveTrainState driveTrainState = mixerStrategy->mix(requestedState.velocity.velocity, requestedState.velocity.angular_velocity, requestedState.velocity.strafe);
        setDriveTrainState(driveTrainState);
    }

    void StateManager::setServoSteeringAngle(const DriveTrainState& driveTrainState, const CONFIG::Handedness side) const {
        servo::Servo *servo;
        float angle;
        float speed;

        if (side == CONFIG::Handedness::LEFT) {
            servo = steering_servos.left;
            angle = driveTrainState.angles.left;
            speed = driveTrainState.speeds[MOTOR_POSITION::FRONT_LEFT];
        } else {
            servo = steering_servos.right;
            angle = driveTrainState.angles.right;
            speed = driveTrainState.speeds[MOTOR_POSITION::FRONT_RIGHT];
        }

        if (std::fabs(speed) > 0.05) {
            if (not(servo->is_enabled())){
                servo->enable();
            }
            servo->value(angle);
        } else {
            servo->disable();
        }
    }

    void StateManager::setDriveTrainState(const DriveTrainState& motorSpeeds) {
        using namespace MOTOR_POSITION;
        stokers[FRONT_LEFT]->set_speed(velocityToRadiansPerSec(motorSpeeds.speeds[FRONT_LEFT]));
        stokers[FRONT_RIGHT]->set_speed(velocityToRadiansPerSec(motorSpeeds.speeds[FRONT_RIGHT]));
        stokers[REAR_LEFT]->set_speed(velocityToRadiansPerSec(motorSpeeds.speeds[REAR_LEFT]));
        stokers[REAR_RIGHT]->set_speed(velocityToRadiansPerSec(motorSpeeds.speeds[REAR_RIGHT]));
        setServoSteeringAngle(motorSpeeds, CONFIG::Handedness::LEFT);
        setServoSteeringAngle(motorSpeeds, CONFIG::Handedness::RIGHT);

        // save the current state
        currentDriveTrainState = motorSpeeds;

        // update the state estimator with the current steering angles
        stateEstimator->updateCurrentSteeringAngles(motorSpeeds.angles);
    }

    float StateManager::velocityToRadiansPerSec(const float velocity) {
        return velocity / (CONFIG::WHEEL_DIAMETER / 2);
    }
} // StateManager