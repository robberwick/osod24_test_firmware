//
// Created by robbe on 04/11/2023.
//

#ifndef OSOD_MOTOR_2040_RECEIVER_SBUS_H
#define OSOD_MOTOR_2040_RECEIVER_SBUS_H

#include "sbus.h"
#include "receiver_utils.h"

enum class SBUS_CHANNELS {
    AIL = 0,
    ELE = 1,
    THR = 2,
    RUD = 3,
    AUX = 4,
    NC = 5
};

const uint32_t receiver_frame_length_ms = 20;

void init_receiver();

ReceiverChannelValues get_channel_values();

bool get_receiver_data();

float map_value_to_range(int value);


#endif //OSOD_MOTOR_2040_RECEIVER_SBUS_H
