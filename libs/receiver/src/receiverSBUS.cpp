//
// Created by robbe on 04/11/2023.
//

#include <cstring>
#include "receiverSBUS.h"

sbus_state_t sbus = {};
uint8_t sbusData[SBUS_MESSAGE_MAX_SIZE] = {};




ReceiverSBUS::ReceiverSBUS(uint8_t pin) : Receiver(pin) {
    // Initialize SBUS and setup interrupt to read data on core0
    sbus_init(SBUS_UART_ID, pin, 16);
}

ReceiverSBUS::~ReceiverSBUS() = default;

ReceiverChannelValues ReceiverSBUS::get_channel_values() const {
    return {
            .AIL = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::AIL)]),
            .ELE = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::ELE)]),
            .THR = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::THR)]),
            .RUD = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::RUD)]),
            .AUX = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::AUX)]),
            .NC = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::NC)]),
    };
}

bool ReceiverSBUS::get_receiver_data() const {
    if (has_sbus_data() && read_sbus_data(sbusData)) {
        memset(&sbus, -1, sizeof(sbus_state_t));

        decode_sbus_data(sbusData, &sbus);

        return true;
    } else {
        return false;
    }
}
