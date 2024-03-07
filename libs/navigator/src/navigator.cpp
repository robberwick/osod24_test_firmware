#include <cstdio>
#include "navigator.h"
#include "statemanager.h"
#include "state_estimator.h"
#include "drivetrain_config.h"

Navigator::Navigator(const Receiver* receiver, STATEMANAGER::StateManager* stateManager, CONFIG::DrivingDirection direction) {
    this->receiver = receiver;
    this->pStateManager = stateManager;
    if (direction == CONFIG::DrivingDirection::SteerableWheelsAtFront){
        driveDirectionFactor = 1;
    } else {
        driveDirectionFactor = -1;
    }
}

void Navigator::navigate() {
    //printf("Navigating...\n");
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

        printf("DriveDirectionFactor: %d\n", driveDirectionFactor);
        // send the receiver data to the state manager
        // TODO: use a queue to send the receiver data to the state manager
        STATE_ESTIMATOR::State requestedState{};
        requestedState.velocity.velocity = driveDirectionFactor * values.ELE * CONFIG::MAX_VELOCITY;
        requestedState.velocity.angular_velocity = values.AIL * CONFIG::MAX_ANGULAR_VELOCITY;
        pStateManager->requestState(requestedState);
    } else {
        printf("No receiver data available\n");
    }
}

Navigator::~Navigator() = default;
