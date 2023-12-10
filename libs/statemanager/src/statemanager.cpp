//
// Created by robbe on 02/12/2023.
//

#include <cstdio>
#include "state_estimator.h"
#include "statemanager.h"
#include "motor2040.hpp"

namespace STATEMANAGER {

    StateManager::StateManager(MIXER::MixerStrategy *mixerStrategy, STATE_ESTIMATOR::StateEstimator *stateEstimator) : mixerStrategy(mixerStrategy), stateEstimator(stateEstimator) {
        printf("State estimator created\n");
        printf("State manager created\n");
        // set up the stokers
        stokers.FRONT_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_A, Direction::NORMAL_DIR);
        stokers.FRONT_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_B, Direction::NORMAL_DIR);
        stokers.REAR_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_C, Direction::NORMAL_DIR);
        stokers.REAR_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_D, Direction::NORMAL_DIR);
    }

    void StateManager::requestState(STATE_ESTIMATOR::State requestedState) {
        printf("Requested state...\n");
        printf("Velocity: %f ", requestedState.velocity);
        printf("Angular velocity: %f ", requestedState.angularVelocity);
        printf("\n");
        MIXER::MotorSpeeds motorSpeeds = mixerStrategy->mix(requestedState.velocity, requestedState.angularVelocity);
        setSpeeds(motorSpeeds);
    }

    void StateManager::setSpeeds(MIXER::MotorSpeeds motorSpeeds) const {
        stokers.FRONT_LEFT->set_speed(motorSpeeds.FRONT_LEFT);
        stokers.FRONT_RIGHT->set_speed(motorSpeeds.FRONT_RIGHT);
        stokers.REAR_LEFT->set_speed(motorSpeeds.REAR_LEFT);
        stokers.REAR_RIGHT->set_speed(motorSpeeds.REAR_RIGHT);
    }
} // StateManager