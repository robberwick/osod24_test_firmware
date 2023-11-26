#include <cstdio>
#include "navigator.h"
#include "pico_cppm/cppm_decoder.h"


Navigator::Navigator(const Receiver &receiver) : receiver(receiver) {

}

void Navigator::navigate() {
    printf("Navigating...\n");
    ReceiverChannelValues values = receiver.get_channel_values();
    printf("AIL: %f ", values.AIL);
    printf("ELE: %f ", values.ELE);
    printf("THR: %f ", values.THR);
    printf("RUD: %f ", values.RUD);
    printf("AUX: %f ", values.AUX);
    printf("NC: %f ", values.NC);
    printf("\n");
}

Navigator::~Navigator() = default;
