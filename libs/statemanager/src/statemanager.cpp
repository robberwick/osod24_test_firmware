//
// Created by robbe on 02/12/2023.
//

#include <cstdio>
#include "state_estimator.h"
#include "statemanager.h"
#include "motor2040.hpp"
#include "servo.hpp"

namespace STATEMANAGER {

    StateManager::StateManager(MIXER::MixerStrategy *mixerStrategy, STATE_ESTIMATOR::StateEstimator *stateEstimator) : mixerStrategy(mixerStrategy), stateEstimator(stateEstimator) {
        printf("State estimator created\n");
        printf("State manager created\n");
        // set up the stokers
        stokers.FRONT_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_A, Direction::NORMAL_DIR);
        stokers.FRONT_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_B, Direction::NORMAL_DIR);
        stokers.REAR_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_C, Direction::NORMAL_DIR);
        stokers.REAR_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_D, Direction::NORMAL_DIR);
        steering_servos.left = new servo::Servo(28); //ADC2 / PWM 6
        steering_servos.right = new servo::Servo(16); //PWM 0
        steering_servos.left->init();
        steering_servos.right->init();
        steering_servos.left->calibration().apply_three_pairs(2200, 1599, 1032, -0.7854, 0, 0.7854);
        steering_servos.right->calibration().apply_three_pairs(1221, 1670, 2200, -0.7854, 0, 0.7854);
        steering_servos.left->enable();
        steering_servos.right->enable();
        steering_servos.left->to_mid();
        steering_servos.right->to_mid();

    }

    void StateManager::requestState(const STATE_ESTIMATOR::State& requestedState) {
        //printf("Requested state...\n");
        //printf("Velocity: %f ", requestedState.velocity);
        //printf("Angular velocity: %f ", requestedState.angularVelocity);
        //printf("\n");
        const COMMON::DriveTrainState driveTrainState = mixerStrategy->mix(requestedState.velocity, requestedState.angularVelocity);
        setSpeeds(driveTrainState);
    }

    void StateManager::setSpeeds(const COMMON::DriveTrainState& motorSpeeds) const {
        stokers.FRONT_LEFT->set_speed(motorSpeeds.speeds.frontLeft);
        stokers.FRONT_RIGHT->set_speed(motorSpeeds.speeds.frontRight);
        stokers.REAR_LEFT->set_speed(motorSpeeds.speeds.rearLeft);
        stokers.REAR_RIGHT->set_speed(motorSpeeds.speeds.rearRight);
        if (std::fabs(motorSpeeds.speeds.frontLeft) > 0.05) {
            if (not(steering_servos.left->is_enabled())){
                steering_servos.left->enable();
            }
            steering_servos.left->value(motorSpeeds.angles.left);
        } else {
            steering_servos.left->disable();
        }
        if (std::fabs(motorSpeeds.speeds.frontRight) > 0.05) {
            if (not(steering_servos.right->is_enabled())){
                steering_servos.right->enable();
            }
            steering_servos.right->value(motorSpeeds.angles.right);
        } else {
            steering_servos.right->disable();
        }
    }
} // StateManager