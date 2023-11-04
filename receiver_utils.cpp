//
// Created by robbe on 25/10/2023.
//
#include "receiver_utils.h"
#include "motor2040.hpp"


float map_range(float a1, float a2, float b1, float b2, float s) {
    return b1 + ((s - a1) * (b2 - b1)) / (a2 - a1);
}

 const char *RX_CHANNEL_NAMES[] = {
        "AIL",
        "ELE",
        "THR",
        "RUD",
        "AUX",
        "NC"
};

 const uint NUM_RX_CHANNELS = count_of(RX_CHANNEL_NAMES);

 const uint RECEIVER_PIN_IN = motor::motor2040::RX_ECHO;