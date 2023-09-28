#include <stdio.h>
#include <cstring>
#include "pico_cppm/cppm_decoder.h"
#include "pico/stdlib.h"
#include "motor2040.hpp"

constexpr uint SYNC_PERIOD_US = 8000; //12800; //20000
constexpr double MIN_PERIOD_US = 1000; //700;
constexpr double MAX_PERIOD_US = 2000; //1600;



/*
Demonstrates how to create multiple Motor objects and control them together.
*/

using namespace motor;
using namespace encoder;

// How many sweeps of the motors to perform
const uint SWEEPS = 2;

// How far from zero to drive the motors when sweeping
constexpr float SPEED_EXTENT = 0.5f; //1.0f;

// Create an array of motor pointers
//const pin_pair motor_pins[] = {motor2040::MOTOR_A, motor2040::MOTOR_B,
//                               motor2040::MOTOR_C, motor2040::MOTOR_D};
const pin_pair motor_pins[] = {motor2040::MOTOR_A};
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
//const pin_pair encoder_pins[] = {motor2040::ENCODER_A, motor2040::ENCODER_B,
//                                 motor2040::ENCODER_C, motor2040::ENCODER_D};
//const char* ENCODER_NAMES[] = {"A", "B", "C", "D"};
const pin_pair encoder_pins[] = {motor2040::ENCODER_A};
const char *ENCODER_NAMES[] = {"A",};
const uint NUM_ENCODERS = count_of(encoder_pins);
Encoder *encoders[NUM_ENCODERS];

/* ======================================================================================== */
// CPPM stuff
/* ======================================================================================== */
constexpr uint CPPM_GPIO_IN = motor2040::ADC0; //26;
const char* CPPM_CHANNEL_NAMES[] = {"AIL", "ELE", "THR", "RUD", "AUX", "NC"};
const uint NUM_CPPM_CHANNELS = count_of(CPPM_CHANNEL_NAMES);

const int motorCPPMChanel[NUM_MOTORS] = {1};


void doCPPMPrint(CPPMDecoder &decoder);

void doEncoderPrint();

// add a function to initialise all motors
void init_motors() {
    // Fill the array of motors and initialise them. Up to 8 motors can be created
    for (auto m = 0u; m < NUM_MOTORS; m++) {
        motors[m] = new Motor(motor_pins[m]);
        motors[m]->init();
    }
}

// add a function to enable all motors
void enable_motors() {
    // Enable all motors
    for (auto m = 0u; m < NUM_MOTORS; m++) {
        motors[m]->enable();
    }
}

void init_encoders() {
    // Fill the array of motors, and initialise them. Up to 8 motors can be created
    for (auto e = 0u; e < NUM_ENCODERS; e++) {
        encoders[e] = new Encoder(pio0, e, encoder_pins[e], PIN_UNUSED, NORMAL_DIR, COUNTS_PER_REV, true);
        encoders[e]->init();
    }
}

int main() {
    stdio_init_all();

    init_motors();
    enable_motors();

    init_encoders();

    sleep_ms(2500);
    printf("Beginning\n");

    CPPMDecoder decoder(CPPM_GPIO_IN, pio1, NUM_CPPM_CHANNELS, SYNC_PERIOD_US, MIN_PERIOD_US, MAX_PERIOD_US);
    CPPMDecoder::sharedInit(0);
    decoder.startListening();

    while (true) {
        // Do a sine speed sweep
        for (auto j = 0u; j < SWEEPS; j++) {
            for (auto i = 0u; i < 360; i++) {
                float speed = sin(((float) i * (float) M_PI) / 180.0f) * SPEED_EXTENT;
                for (auto m = 0u; m < NUM_MOTORS; m++) {
                    motors[m]->speed((float) decoder.getChannelValue(motorCPPMChanel[m]));
                }
                doCPPMPrint(decoder);
                doEncoderPrint();
                sleep_ms(20);
            }
        }


    }
}

void doEncoderPrint() {
    for (auto e = 0u; e < NUM_ENCODERS; e++) {
        printf("%s = %ld, ", ENCODER_NAMES[e], encoders[e]->count());
    }
    printf("\n");
}

void doCPPMPrint(CPPMDecoder &decoder) {
    for (int i=0; i < NUM_CPPM_CHANNELS; i++) {
        if (strcmp(CPPM_CHANNEL_NAMES[i], "NC") != 0) {
            printf(
                    "%s: %ld / %.2f, ",
                    CPPM_CHANNEL_NAMES[i],
                    (long) decoder.getChannelUs(i),
                    decoder.getChannelValue(i)
                    );
        }
    }
    printf("ERRS: %lu ", decoder.getFrameErrorCount());
    printf("AGE: %lu", decoder.getFrameAgeMs());
    printf("\n");
}
