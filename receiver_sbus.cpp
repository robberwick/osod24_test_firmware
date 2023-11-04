//
// Created by robbe on 04/11/2023.
//

#include <cstring>
#include "receiver_sbus.h"

sbus_state_t sbus = {};
uint8_t sbusData[SBUS_MESSAGE_MAX_SIZE] = {};


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
    sbus_init(SBUS_UART_ID, RECEIVER_PIN_IN, 16);
}

ReceiverChannelValues get_channel_values() {
    return {
            .AIL = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::AIL)]),
            .ELE = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::ELE)]),
            .THR = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::THR)]),
            .RUD = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::RUD)]),
            .AUX = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::AUX)]),
            .NC = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::NC)]),
    };
}

bool get_receiver_data() {
    if (has_sbus_data() && read_sbus_data(sbusData)) {
            memset(&sbus, -1, sizeof(sbus_state_t));

            decode_sbus_data(sbusData, &sbus);

            return true;
        } else {
            return false;
        }
}