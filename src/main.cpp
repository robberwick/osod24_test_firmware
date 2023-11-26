#include <cstdio>
#include "pico/stdlib.h"
#include "navigator.h"
#include "receiver.h"
#include "motor2040.hpp"


int main() {
    stdio_init_all();

    Receiver receiver = Receiver(motor::motor2040::RX_ECHO);
    Navigator navigator = Navigator(receiver);

    while (true) {
        navigator.navigate();
        sleep_ms(20);
    }
}
