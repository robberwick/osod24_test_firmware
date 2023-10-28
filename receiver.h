//
// Created by robbe on 25/10/2023.
//

#ifndef OSOD_MOTOR_2040_RECEIVER_H
#define OSOD_MOTOR_2040_RECEIVER_H

#ifdef RX_PROTOCOL_SBUS
#include "sbus.h"
#elif defined(RX_PROTOCOL_CPPM)
#include "pico_cppm/cppm_decoder.h"
#else
#error You must define either RX_PROTOCOL_SBUS or RX_PROTOCOL_CPPM
#endif

struct ReceiverChannelValues {
    float AIL;
    float ELE;
    float THR;
    float RUD;
    float AUX;
    float NC;
};

float map_range(float a1, float a2, float b1, float b2, float s);

float map_value_to_range(int value);

void init_receiver();

#endif //OSOD_MOTOR_2040_RECEIVER_H
