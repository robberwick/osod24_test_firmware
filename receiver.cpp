//
// Created by robbe on 25/10/2023.
//
#include "receiver.h"


float map_range(float a1, float a2, float b1, float b2, float s) {
    return b1 + ((s - a1) * (b2 - b1)) / (a2 - a1);
}

float map_value_to_range(int value) {
    float mapped_value;
    float fvalue = static_cast<float>(value);

    if (fvalue < 1024.0f) {
        mapped_value = map_range(240.0f, 1024.0f, -1.0f, 0.0f, fvalue);
    } else {
        mapped_value = map_range(1024.0f, 1807.0f, 0.0f, 1.0f, fvalue);
    }

    return mapped_value;
}

void init_receiver() {
    // Initialize SBUS and setup interrupt to read data on core0
    sbus_init(SBUS_UART_ID, 17, 16);
}
