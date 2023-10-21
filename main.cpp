#include <cstdio>
#include <cstring>
#include "pico_cppm/cppm_decoder.h"
#include "pico/stdlib.h"
#include "motor2040.hpp"
#include "tank_driver_mixer.h"
#include "sbus.h"

constexpr uint SYNC_PERIOD_US = 8000; //12800; //20000
constexpr double MIN_PERIOD_US = 1000; //700;
constexpr double MAX_PERIOD_US = 2000; //1600;



/*
Demonstrates how to create multiple Motor objects and control them together.
*/

using namespace motor;
using namespace encoder;

// max speed factor - scale the speed of the motors down to this value
constexpr float SPEED_EXTENT = 1.0f;

// Create an array of motor pointers
const pin_pair motor_pins[] = {motor2040::MOTOR_A, motor2040::MOTOR_B,
                               motor2040::MOTOR_C, motor2040::MOTOR_D};
const uint NUM_MOTORS = count_of(motor_pins);
Motor *motors[NUM_MOTORS];
enum MOTOR_NAMES {
    LEFT_FRONT = 0,
    RIGHT_FRONT = 1,
    LEFT_REAR = 2,
    RIGHT_REAR = 3,
};

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
const char *ENCODER_NAMES[] = {"A", "B", "C", "D"};
const uint NUM_ENCODERS = count_of(encoder_pins);
Encoder *encoders[NUM_ENCODERS];

/* ======================================================================================== */
// RC Receiver stuff
/* ======================================================================================== */

constexpr uint RC_RECV_GPIO_IN = motor2040::ADC0; //26;

/* ======================================================================================== */
// CPPM stuff
/* ======================================================================================== */
enum class CPPM_CHANNELS {
    AIL = 0,
    ELE = 1,
    THR = 2,
    RUD = 3,
    AUX = 4,
    NC = 5
};
const char *CPPM_CHANNEL_NAMES[] = {
        "AIL",
        "ELE",
        "THR",
        "RUD",
        "AUX",
        "NC"
};
const uint NUM_CPPM_CHANNELS = count_of(CPPM_CHANNEL_NAMES);

/* ======================================================================================== */
// SBUS stuff
/* ======================================================================================== */
sbus_state_t sbus = {};
uint8_t sbusData[SBUS_MESSAGE_MAX_SIZE] = {};
const uint32_t interval_ms = 20;

enum class SBUS_CHANNELS {
    AIL = 0,
    ELE = 1,
    THR = 2,
    RUD = 3,
    AUX = 4,
    NC = 5
};
const char *SBUS_CHANNEL_NAMES[] = {
        "AIL",
        "ELE",
        "THR",
        "RUD",
        "AUX",
        "NC"
};
const uint NUM_SBUS_CHANNELS = count_of(CPPM_CHANNEL_NAMES);


void doCPPMPrint(CPPMDecoder &decoder);

void doEncoderPrint();

// add a function to initialise all motors
void init_motors() {
    // Fill the motors array and initialise them. Up to 8 motors can be created
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
    // Fill the encoder array, and initialise them. Up to 8 encoders can be created
    for (auto e = 0u; e < NUM_ENCODERS; e++) {
        encoders[e] = new Encoder(pio0, e, encoder_pins[e], PIN_UNUSED, NORMAL_DIR, COUNTS_PER_REV, true);
        encoders[e]->init();
    }
}

float map_range(float a1, float a2, float b1, float b2, float s)
{
    return b1 + ((s - a1) * (b2 - b1)) / (a2 - a1);
}
float map_value_to_range(int value)
{
    float mapped_value;
    float fvalue = static_cast<float>(value);

    if (fvalue < 1024.0f) {
        mapped_value = map_range(240.0f, 1024.0f, -1.0f, 0.0f, fvalue);
    }
    else {
        mapped_value = map_range(1024.0f, 1807.0f, 0.0f, 1.0f, fvalue);
    }

    return mapped_value;
}

int main() {
    stdio_init_all();

    init_motors();
    enable_motors();

    init_encoders();



    sleep_ms(2500);
    printf("Beginning\n");

    // Initialize SBUS and setup interrupt to read data on core0
    sbus_init(SBUS_UART_ID, 17, 16);

//    CPPMDecoder decoder(RC_RECV_GPIO_IN, pio1, NUM_CPPM_CHANNELS, SYNC_PERIOD_US, MIN_PERIOD_US, MAX_PERIOD_US);
//    CPPMDecoder::sharedInit(0);
//    decoder.startListening();

    uint32_t start_ms = millis();

    while (true) {
//        doCPPMPrint(decoder);
        doEncoderPrint();
        if (hasSbusData()) {
            if (readSbusData(sbusData)) {
                memset(&sbus, -1, sizeof(sbus_state_t));

                decode_sbus_data(sbusData, &sbus);

                if (millis() - start_ms > interval_ms) {
                    start_ms = millis();

                    for (int i = 0; i < 18; ++i) {
                        printf("Ch%2i: %04u %0.2f ", i, sbus.ch[i], map_value_to_range(sbus.ch[i]));
                    }
                    printf("\n");

                    // Get the aileron channel value for steering
                    //        auto steering = (float) decoder.getChannelValue(CPPM_CHANNELS::AIL);
                    auto steering = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::AIL)]);
                    // Get the elevator channel value for throttle
                    //        auto throttle = (float) decoder.getChannelValue(CPPM_CHANNELS::ELE);
                    auto throttle = (float) map_value_to_range(sbus.ch[static_cast<int>(SBUS_CHANNELS::ELE)]);
                    MotorSpeed speed = tank_steer_mix(steering, throttle, SPEED_EXTENT);

                    motors[MOTOR_NAMES::LEFT_FRONT]->speed(speed.left);
                    motors[MOTOR_NAMES::LEFT_REAR]->speed(speed.left);
                    motors[MOTOR_NAMES::RIGHT_FRONT]->speed(speed.right);
                    motors[MOTOR_NAMES::RIGHT_REAR]->speed(speed.right);


//                    printf("Frame lost: %i Failsafe: %i\n", sbus.framelost, sbus.failsafe);
                }
            }
        }


        sleep_ms(20);
    }
}

void doEncoderPrint() {
    for (auto e = 0u; e < NUM_ENCODERS; e++) {
        printf("%s = %ld, ", ENCODER_NAMES[e], encoders[e]->count());
    }
    printf("\n");
}

void doCPPMPrint(CPPMDecoder &decoder) {
    for (int i = 0; i < NUM_CPPM_CHANNELS; i++) {
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
