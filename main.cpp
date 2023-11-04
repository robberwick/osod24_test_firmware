#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "motor2040.hpp"
#include "tank_driver_mixer.h"

#if defined( RX_PROTOCOL_SBUS)
#include "receiver_sbus.h"
#elif defined(RX_PROTOCOL_CPPM)
#include "receiver_cppm.h"
#include "pico_cppm/cppm_decoder.h"
#else
#error You must define either RX_PROTOCOL_SBUS or RX_PROTOCOL_CPPM
#endif


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

//const char *SBUS_CHANNEL_NAMES[] = {
//        "AIL",
//        "ELE",
//        "THR",
//        "RUD",
//        "AUX",
//        "NC"
//};
//const uint NUM_SBUS_CHANNELS = count_of(CPPM_CHANNEL_NAMES);


//void doCPPMPrint(CPPMDecoder &decoder);

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
    for (auto & motor : motors) {
        motor->enable();
    }
}

void init_encoders() {
    // Fill the encoder array, and initialise them. Up to 8 encoders can be created
    for (auto e = 0u; e < NUM_ENCODERS; e++) {
        encoders[e] = new Encoder(pio0, e, encoder_pins[e], PIN_UNUSED, NORMAL_DIR, COUNTS_PER_REV, true);
        encoders[e]->init();
    }
}


int main() {
    stdio_init_all();

    init_receiver();

    init_motors();
    enable_motors();

    init_encoders();


    sleep_ms(2500);
    printf("Beginning\n");

    uint32_t start_ms = millis();

    while (true) {
//        doCPPMPrint(decoder);
        doEncoderPrint();
        if (get_receiver_data()) {
            ReceiverChannelValues rxValues = get_channel_values();

            if (millis() - start_ms > receiver_frame_length_ms) {
                start_ms = millis();

                printf("AIL: %0.2f ELE: %0.2f THR: %0.2f RUD: %0.2f AUX: %0.2f NC: %0.2f\n",
                       rxValues.AIL,
                       rxValues.ELE,
                       rxValues.THR,
                       rxValues.RUD,
                       rxValues.AUX,
                       rxValues.NC
                );

                // Get the aileron channel value for steering
                //        auto steering = (float) decoder.getChannelValue(CPPM_CHANNELS::AIL);
                auto steering = rxValues.AIL;
                // Get the elevator channel value for throttle
                //        auto throttle = (float) decoder.getChannelValue(CPPM_CHANNELS::ELE);
                auto throttle = rxValues.ELE;
                MotorSpeed speed = tank_steer_mix(steering, throttle, SPEED_EXTENT);

                motors[MOTOR_NAMES::LEFT_FRONT]->speed(speed.left);
                motors[MOTOR_NAMES::LEFT_REAR]->speed(speed.left);
                motors[MOTOR_NAMES::RIGHT_FRONT]->speed(speed.right);
                motors[MOTOR_NAMES::RIGHT_REAR]->speed(speed.right);
            }
        }


        sleep_ms(receiver_frame_length_ms);
    }
}

void doEncoderPrint() {
    for (auto e = 0u; e < NUM_ENCODERS; e++) {
        printf("%s = %ld, ", ENCODER_NAMES[e], encoders[e]->count());
    }
    printf("\n");
}

//void doCPPMPrint(CPPMDecoder &decoder) {
//    for (int i = 0; i < NUM_CPPM_CHANNELS; i++) {
//        if (strcmp(CPPM_CHANNEL_NAMES[i], "NC") != 0) {
//            printf(
//                    "%s: %ld / %.2f, ",
//                    CPPM_CHANNEL_NAMES[i],
//                    (long) decoder.getChannelUs(i),
//                    decoder.getChannelValue(i)
//            );
//        }
//    }
//    printf("ERRS: %lu ", decoder.getFrameErrorCount());
//    printf("AGE: %lu", decoder.getFrameAgeMs());
//    printf("\n");
//}
