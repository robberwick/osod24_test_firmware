#include <cstdio>
#include "navigator.h"
#include "statemanager.h"


Navigator::Navigator(const Receiver* receiver, STATEMANAGER::StateManager* stateManager) {
    this->receiver = receiver;
    this->pStateManager = stateManager;

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

        // send the receiver data to the state manager
        // TODO: use a queue to send the receiver data to the state manager
        STATEMANAGER::RequestedState requestedState{};
        requestedState.velocity = values.ELE;
        requestedState.angularVelocity = values.AIL;
        pStateManager->requestState(requestedState);
    } else {
        printf("No receiver data available\n");
    }
}

Navigator::~Navigator() = default;
