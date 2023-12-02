//
// Created by robbe on 02/12/2023.
//

#include <cstdio>
#include "statemanager.h"

namespace STATEMANAGER {


    StateManager::StateManager() = default;

    void StateManager::requestState(RequestedState requestedState) {
        printf("Requested state...\n");
        printf("Velocity: %f ", requestedState.velocity);
        printf("Angular velocity: %f ", requestedState.angularVelocity);
        printf("\n");

    }
} // StateManager