#include <stdio.h>
#include "pico_cppm/cppm_decoder.h"
#include "pico/stdlib.h"
#include "motor2040.hpp"

constexpr uint CPPM_GPIO_IN = 26;
constexpr uint SYNC_PERIOD_US = 8000; //12800; //20000
constexpr double MIN_PERIOD_US = 1000; //700;
constexpr double MAX_PERIOD_US = 2000; //1600;
constexpr uint NUM_CHANNELS = 6;

/*
Demonstrates how to create multiple Motor objects and control them together.
*/

using namespace motor;
using namespace encoder;

// How many sweeps of the motors to perform
const uint SWEEPS = 2;

// The number of discrete sweep steps
const uint STEPS = 10;

// The time in milliseconds between each step of the sequence
const uint STEPS_INTERVAL_MS = 500;

// How far from zero to drive the motors when sweeping
constexpr float SPEED_EXTENT = 1.0f;

// Create an array of motor pointers
const pin_pair motor_pins[] = {motor2040::MOTOR_A, motor2040::MOTOR_B,
                               motor2040::MOTOR_C, motor2040::MOTOR_D};
const uint NUM_MOTORS = count_of(motor_pins);
Motor *motors[NUM_MOTORS];

/* ======================================================================================== */
// encoder stuff
/* ======================================================================================== */

// The gear ratio of the motor
constexpr float GEAR_RATIO = 50.0f;

// The counts per revolution of the motor's output shaft
constexpr float COUNTS_PER_REV = MMME_CPR * GEAR_RATIO;


// Create an array of encoder pointers
const pin_pair encoder_pins[] = {motor2040::ENCODER_A, motor2040::ENCODER_B,
                                 motor2040::ENCODER_C, motor2040::ENCODER_D};
const char* ENCODER_NAMES[] = {"A", "B", "C", "D"};
const uint NUM_ENCODERS = count_of(encoder_pins);
Encoder *encoders[NUM_ENCODERS];


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