#include "receiverCPPM.h"
#include "receiver.h"


const char *RX_CHANNEL_NAMES[] = {
        "AIL",
        "ELE",
        "THR",
        "RUD",
        "AUX",
        "NC"
};

const uint NUM_RX_CHANNELS = count_of(RX_CHANNEL_NAMES);
constexpr uint SYNC_PERIOD_US = 8000; //12800; //20000
constexpr double MIN_PERIOD_US = 1008; //700;
constexpr double MAX_PERIOD_US = 2000; //1600;

ReceiverCPPM::ReceiverCPPM(uint8_t pin) {
    // Initialize CPPM and setup interrupt to read data on core0
    decoder = new CPPMDecoder(pin, pio1, NUM_RX_CHANNELS, SYNC_PERIOD_US, MIN_PERIOD_US,
                                   MAX_PERIOD_US);;
    CPPMDecoder::sharedInit(0);
    decoder->startListening();
}

ReceiverChannelValues ReceiverCPPM::get_channel_values() const {
    return {
            .AIL = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::AIL)),
            .ELE = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::ELE)),
            .THR = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::THR)),
            .RUD = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::RUD)),
            .AUX = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::AUX)),
            .NC = (float) decoder->getChannelValue(static_cast<int>(RX_CHANNELS::NC)),
    };
}

bool ReceiverCPPM::get_receiver_data() const {
    return decoder->getFrameAgeUs() > 0;
}

