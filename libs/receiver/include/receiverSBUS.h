//
// Created by robbe on 04/11/2023.
//

#ifndef OSOD_MOTOR_2040_RECEIVER_SBUS_H
#define OSOD_MOTOR_2040_RECEIVER_SBUS_H

#include "sbus_2040.h"
#include "receiver.h"

enum class SBUS_CHANNELS {
    AIL = 0,
    ELE = 1,
    THR = 2,
    RUD = 3,
    AUX = 4,
    NC = 5
};

const uint32_t receiver_frame_length_ms = 20;

class ReceiverSBUS : public Receiver {
public:
    // constructor
    explicit ReceiverSBUS(uint8_t pin);

    // destructor
    ~ReceiverSBUS() override;

    [[nodiscard]] ReceiverChannelValues get_channel_values() const override;

    bool get_receiver_data() const override;
};

#endif //OSOD_MOTOR_2040_RECEIVER_SBUS_H