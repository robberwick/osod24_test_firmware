#include <cstdio>
#include "navigator.h"
#include "statemanager.h"
#include "state_estimator.h"
#include "types.h"
#include "drivetrain_config.h"
#include "waypoint_navigation.h"

Navigator::Navigator(const Receiver* receiver, STATEMANAGER::StateManager* stateManager, CONFIG::SteeringStyle direction) {
    this->receiver = receiver;
    this->pStateManager = stateManager;

    driveDirection = direction;
    navigationMode = NAVIGATION_MODE::REMOTE_CONTROL;
}

void Navigator::navigate() {
    printf("Navigating...\n");
    if (receiver->get_receiver_data()) {
        //printf("Receiver data available\n");

        ReceiverChannelValues values = receiver->get_channel_values();
        //printf("AIL: %f ", values.AIL);
        //printf("ELE: %f ", values.ELE);
        //printf("THR: %f ", values.THR);
        //printf("RUD: %f ", values.RUD);
        //printf("AUX: %f ", values.AUX);
        //printf("NC: %f ", values.NC);
        //printf("\n");

        NAVIGATION_MODE::Mode newMode;
        newMode = determineMode(values.AUX);
        if (newMode != navigationMode){
            printf("changing mode to mode %d, where 1=RC, 2=waypoint, 3=Pi\n", newMode);
            navigationMode = newMode;
        }
        STATE_ESTIMATOR::VehicleState requestedState{};
        if (shouldResetWaypointIndex(values.THR)){
            printf("resetting waypoint index to 0.\n");
            waypointNavigator.targetWaypointIndex = 0;
        }
        switch (navigationMode) {
        case NAVIGATION_MODE::WAYPOINT:
            waypointNavigator.navigate(current_state);
            requestedState.velocity.velocity = waypointNavigator.desiredV;
            requestedState.velocity.angular_velocity = waypointNavigator.desiredW;
            break;
        default: //includes REMOTE_CONTROL, which is the default
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

NAVIGATION_MODE::Mode Navigator::determineMode(float signal){
    NAVIGATION_MODE::Mode mode;
    if (signal > waypointModeThreshold){
        mode = NAVIGATION_MODE::WAYPOINT;
    } else {
        mode = NAVIGATION_MODE::REMOTE_CONTROL;
    }
    return mode;
}

bool Navigator::shouldResetWaypointIndex(float signal){
    return (signal > waypointIndexThreshold);
}

void Navigator::update(const VehicleState newState) {
    current_state = newState;
}

Navigator::~Navigator() = default;
