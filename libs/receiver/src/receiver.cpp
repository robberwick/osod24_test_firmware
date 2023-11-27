#include "receiver.h"
#include "motor2040.hpp"

#ifdef RX_PROTOCOL_CPPM
#include "receiverCPPM.h"
#else
#include "receiverSBUS.h"
#endif


constexpr uint SYNC_PERIOD_US = 8000; //12800; //20000
constexpr double MIN_PERIOD_US = 1000; //700;
constexpr double MAX_PERIOD_US = 2000; //1600;

Receiver* getReceiver(int pin) {
#ifdef RX_PROTOCOL_CPPM
    auto* receiver = new ReceiverCPPM(motor::motor2040::RX_ECHO);
#else
    auto* receiver = new ReceiverSBUS(motor::motor2040::RX_ECHO);
#endif
    return receiver;
}

Receiver::Receiver(int pin) {
}

Receiver::Receiver() = default;

Receiver::~Receiver() = default;

ReceiverChannelValues Receiver::get_channel_values() const {
    return {};
}

bool Receiver::get_receiver_data() const{
    return false;
}

float map_value_to_range(int value) {
    float mapped_value;
    auto fvalue = static_cast<float>(value);

    if (fvalue < 1024.0f) {
        mapped_value = map_range(240.0f, 1024.0f, -1.0f, 0.0f, fvalue);
    } else {
        mapped_value = map_range(1024.0f, 1807.0f, 0.0f, 1.0f, fvalue);
    }

    return mapped_value;
}

float map_range(float a1, float a2, float b1, float b2, float s) {
    return b1 + ((s - a1) * (b2 - b1)) / (a2 - a1);
}
