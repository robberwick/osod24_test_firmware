//
// Created by robbe on 02/12/2023.
//

#include <cstdio>
#include "statemanager.h"
#include "motor2040.hpp"

namespace STATEMANAGER {

    StateManager::StateManager() {
        printf("State manager created\n");
        // set up the stokers
        stokers.FRONT_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_A, Direction::NORMAL_DIR);
        stokers.FRONT_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_B, Direction::NORMAL_DIR);
        stokers.REAR_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_C, Direction::NORMAL_DIR);
        stokers.REAR_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_D, Direction::NORMAL_DIR);
    };

    void StateManager::requestState(RequestedState requestedState) {
        printf("Requested state...\n");
        printf("Velocity: %f ", requestedState.velocity);
        printf("Angular velocity: %f ", requestedState.angularVelocity);
        printf("\n");
        mixTankDrive(&requestedState);
    }

    void StateManager::mixTankDrive(RequestedState *requestedState) const {
        float left = requestedState->velocity + requestedState->angularVelocity;
        float right = requestedState->velocity - requestedState->angularVelocity;
        float abs_left = left >= 0 ? left : -left;
        float abs_right = right >= 0 ? right : -right;
        float max_abs = abs_left > abs_right ? abs_left : abs_right;

        if (max_abs > 1.0) {
            left /= max_abs;
            right /= max_abs;
        }

        float scaled_left = (left * -SPEED_EXTENT);
        float scaled_right = (right * -SPEED_EXTENT) * -1;

        stokers.FRONT_LEFT->set_speed(scaled_left);
        stokers.FRONT_RIGHT->set_speed(scaled_right);
        stokers.REAR_LEFT->set_speed(scaled_left);
        stokers.REAR_RIGHT->set_speed(scaled_right);
    }
} // StateManager