#include <cstdio>
#include "navigator.h"
#include "statemanager.h"
#include "state_estimator.h"
#include "types.h"
#include "drivetrain_config.h"

Navigator::Navigator(const Receiver* receiver, STATEMANAGER::StateManager* stateManager) {
    this->receiver = receiver;
    this->pStateManager = stateManager;
    navigationMode = COMMON::REMOTE_CONTROL;
}

void Navigator::navigate() {
    //printf("Navigating...\n");
    if (receiver->get_receiver_data()) {
        //printf("Receiver data available\n");
        ReceiverChannelValues values = receiver->get_channel_values();

        COMMON::NavigationMode newMode;
        newMode = determineMode(values.THR);
        if (newMode != navigationMode){
            printf("changing mode to mode %d, where 1=RC, 2=waypoint, 3=Pi\n", newMode);
            navigationMode = newMode;
        }
        STATE_ESTIMATOR::State requestedState{};

        switch (navigationMode) {
        case COMMON::REMOTE_CONTROL:
            requestedState.velocity.velocity = values.ELE * CONFIG::MAX_VELOCITY;
            requestedState.velocity.angular_velocity = values.AIL * CONFIG::MAX_ANGULAR_VELOCITY;
            break;
        case COMMON::WAYPOINT:
            requestedState.velocity.velocity = 0;
            requestedState.velocity.angular_velocity = 0;
            break;
        default:
            requestedState.velocity.velocity = values.ELE * CONFIG::MAX_VELOCITY;
            requestedState.velocity.angular_velocity = values.AIL * CONFIG::MAX_ANGULAR_VELOCITY;
            break;
        }
        // send the receiver data to the state manager
        // TODO: use a queue to send the receiver data to the state manager
        pStateManager->requestState(requestedState);
    } else {
        printf("No receiver data available\n");
    }
}

COMMON::NavigationMode Navigator::determineMode(float signal){
    COMMON::NavigationMode mode;
    if (signal > waypointSignalThreshold){
        mode = COMMON::WAYPOINT;
    } else {
        mode = COMMON::REMOTE_CONTROL;
    }
    return mode;
}

Navigator::~Navigator() = default;
