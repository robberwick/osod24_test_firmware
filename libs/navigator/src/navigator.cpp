#include <cstdio>
#include "navigator.h"


Navigator::Navigator(const Receiver* receiver) {
    this->receiver = receiver;

}

void Navigator::navigate() {
    printf("Navigating...\n");
    if (receiver->get_receiver_data()) {
        printf("Receiver data available\n");
        ReceiverChannelValues values = receiver->get_channel_values();
        printf("AIL: %f ", values.AIL);
        printf("ELE: %f ", values.ELE);
        printf("THR: %f ", values.THR);
        printf("RUD: %f ", values.RUD);
        printf("AUX: %f ", values.AUX);
        printf("NC: %f ", values.NC);
        printf("\n");
    } else {
        printf("No receiver data available\n");
    }
}

Navigator::~Navigator() = default;
