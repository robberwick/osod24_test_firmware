//
// Created by robbe on 25/10/2023.
//
#ifndef OSOD_MOTOR_2040_RECEIVER_H
#define OSOD_MOTOR_2040_RECEIVER_H

#include <cstdio>
#include "pico/stdlib.h"

extern const uint RECEIVER_PIN_IN;

struct ReceiverChannelValues {
    float AIL;
    float ELE;
    float THR;
    float RUD;
    float AUX;
    float NC;
};

enum class RX_CHANNELS {
    AIL = 0,
    ELE = 1,
    THR = 2,
    RUD = 3,
    AUX = 4,
    NC = 5
};
extern const char *RX_CHANNEL_NAMES[];

extern const uint NUM_RX_CHANNELS;

float map_range(float a1, float a2, float b1, float b2, float s);


#endif //OSOD_MOTOR_2040_RECEIVER_H
