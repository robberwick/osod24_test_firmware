//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_STATEMANAGER_H
#define OSOD_MOTOR_2040_STATEMANAGER_H

#include "receiver.h"
#include "stoker.h"

namespace STATEMANAGER {

    // define a RequestedState struct containing the requested state parameters: velocity and angular velocity
    struct RequestedState {
        float velocity;
        float angularVelocity;
    };

    struct Stokers {
        STOKER::Stoker* FRONT_LEFT;
        STOKER::Stoker* FRONT_RIGHT;
        STOKER::Stoker* REAR_LEFT;
        STOKER::Stoker* REAR_RIGHT;
    };

    class StateManager {
    public:
        StateManager();

        void requestState(RequestedState requestedState);
    private:
        Stokers stokers{};
        // max speed factor - scale the speed of the motors down to this value
        static constexpr float SPEED_EXTENT = 1.0f;

        void mixTankDrive(RequestedState *requestedState) const;
    };

} // StateManager

#endif //OSOD_MOTOR_2040_STATEMANAGER_H
