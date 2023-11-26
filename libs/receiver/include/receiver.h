#pragma once

#include "pico_cppm/cppm_decoder.h"

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

class Receiver {
public:
    // constructor
    explicit Receiver(uint8_t pin);
    // destructor
    ~Receiver();

    ReceiverChannelValues get_channel_values();
    bool get_receiver_data();

private:
    CPPMDecoder* decoder = nullptr;
};



