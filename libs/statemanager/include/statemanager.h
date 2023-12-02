//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_STATEMANAGER_H
#define OSOD_MOTOR_2040_STATEMANAGER_H

#include "receiver.h"

namespace STATEMANAGER {

    // define a RequestedState struct containing the requested state parameters: velocity and angular velocity
    struct RequestedState {
        float velocity;
        float angularVelocity;
    };

    class StateManager {
    public:
        StateManager();

        void requestState(RequestedState requestedState);
    };

} // StateManager

#endif //OSOD_MOTOR_2040_STATEMANAGER_H
