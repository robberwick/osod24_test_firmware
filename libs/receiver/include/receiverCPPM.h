#pragma once

#include "pico_cppm/cppm_decoder.h"
#include "receiver.h"



class ReceiverCPPM: public Receiver {
public:
    // constructor
    explicit ReceiverCPPM(uint8_t pin);

    [[nodiscard]] ReceiverChannelValues get_channel_values() const override;

    bool get_receiver_data() const override;

private:
    CPPMDecoder *decoder = nullptr;
};

