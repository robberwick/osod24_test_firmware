#include <cstdio>
#include "pico/stdlib.h"
#include "navigator.h"
#include "receiver.h"
#include "motor2040.hpp"


int main() {
    stdio_init_all();

    // if the cmake build flag RX_PROTOCOL is CPPM, then use the CPPM receiver
    // otherwise use the SBUS receiver
    Receiver* pReceiver = getReceiver(motor::motor2040::RX_ECHO);
    Navigator navigator = Navigator(pReceiver);

    while (true) {
        navigator.navigate();
        sleep_ms(20);
    }
}
