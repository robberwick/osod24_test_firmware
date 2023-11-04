//
// Created by robbe on 04/11/2023.
//

#include <cstdio>
#include <hardware/pio.h>
#include "receiver_cppm.h"
#include "pico_cppm/cppm_decoder.h"


constexpr uint SYNC_PERIOD_US = 8000; //12800; //20000
constexpr double MIN_PERIOD_US = 1000; //700;
constexpr double MAX_PERIOD_US = 2000; //1600;



CPPMDecoder *decoder;

void init_receiver() {
    // Initialize CPPM and setup interrupt to read data on core0
        decoder = new CPPMDecoder(RECEIVER_PIN_IN, pio1, NUM_RX_CHANNELS, SYNC_PERIOD_US, MIN_PERIOD_US, MAX_PERIOD_US);;
        CPPMDecoder::sharedInit(0);
        decoder->startListening();
}

ReceiverChannelValues get_channel_values() {
    return {
            .AIL = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::AIL)),
            .ELE = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::ELE)),
            .THR = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::THR)),
            .RUD = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::RUD)),
            .AUX = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::AUX)),
            .NC = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::NC)),
    };
}

bool get_receiver_data() {
    return decoder->getFrameAgeMs() > 0;
}