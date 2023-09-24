#include <stdio.h>
#include "pico_cppm/cppm_decoder.h"
#include "pico/stdlib.h"

constexpr uint CPPM_GPIO_IN = 26;
constexpr uint SYNC_PERIOD_US = 8000; //12800; //20000
constexpr double MIN_PERIOD_US = 1000; //700;
constexpr double MAX_PERIOD_US = 2000; //1600;
constexpr uint NUM_CHANNELS = 6;

int main() {
    stdio_init_all();

    sleep_ms(2500);
    printf("Beginning\n");

    CPPMDecoder decoder(CPPM_GPIO_IN, pio1, NUM_CHANNELS, SYNC_PERIOD_US, MIN_PERIOD_US, MAX_PERIOD_US);
    CPPMDecoder::sharedInit(0);
    decoder.startListening();

    while (true) {
        printf("AIL: %ld ELE: %ld THR: %ld RUD: %ld AUX: %ld NC: %ld ERRS: %lu AGE: %lu\n",
               (long) decoder.getChannelUs(0),
               (long) decoder.getChannelUs(1),
               (long) decoder.getChannelUs(2),
               (long) decoder.getChannelUs(3),
               (long) decoder.getChannelUs(4),
               (long) decoder.getChannelUs(5),
               decoder.getFrameErrorCount(),
               decoder.getFrameAgeMs()
        );
    }
}