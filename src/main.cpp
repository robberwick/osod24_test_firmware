#include <cstdio>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "navigator.h"
#include "receiver.h"
#include "statemanager.h"
#include "state_estimator.h"
#include "motor2040.hpp"
#include "tank_steer_strategy.h"
#include "ackermann_strategy.h"
#include "drivetrain_config.h"
#include "utils.h"
#include "tf_luna.h"
#include "bno080.h"

Navigator *navigator;
bool shouldNavigate = false;

extern "C" void timer_callback(repeating_timer_t *t) {
    // Call the navigate function in the interrupt handler
    shouldNavigate = true;
}

int main() {
    stdio_init_all();
    i2c_inst_t* i2c_port0;
    initI2C(i2c_port0, 100 * 1000, CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN);

    i2c_inst_t* i2c_port0;
    initI2C(i2c_port0, 100 * 1000, CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN);

    //set up IMU
    BNO08x IMU;
    if (IMU.begin(CONFIG::BNO08X_ADDR, i2c_port0)==false) {
        while (1){
            printf("BNO08x not detected at default I2C address. Check wiring. Freezing\n");
            sleep_ms(1000);
        }
    }
    if (IMU.enableRotationVector() == true) {
        printf("Rotation vector enabled\n");
    } else {
        printf("Could not enable rotation vector\n");
    }

    // set up the state estimator
    auto *pStateEstimator = new STATE_ESTIMATOR::StateEstimator(&IMU);

    // set up the state manager
    using namespace STATEMANAGER;

    auto* pAckermannSteerStrategy = new MIXER::AckermannMixer(CONFIG::WHEEL_TRACK, CONFIG::WHEEL_BASE);
    auto* pStateManager = new StateManager(pAckermannSteerStrategy, pStateEstimator);


    // set up the receiver
    // if the cmake build flag RX_PROTOCOL is CPPM, then use the CPPM receiver
    // otherwise use the SBUS receiver
    Receiver *pReceiver = getReceiver(motor::motor2040::RX_ECHO);

    // set up the navigator
    navigator = new Navigator(pReceiver, pStateManager);

    // Initialize a hardware timer
    repeating_timer_t navigationTimer;
    add_repeating_timer_ms(
            20,
            reinterpret_cast<repeating_timer_callback_t>(timer_callback),
            nullptr,
            &navigationTimer
    );

    while (true) {
        // Do nothing in the main loop
        LidarData frontLidarData = getLidarData(tf_luna_front, i2c_port0); // Get and process radar data
        printf("front: distance = %5dcm, strength = %5d, temperature = %5d°C\n",
               frontLidarData.distance, frontLidarData.strength, frontLidarData.temperature);
        LidarData rightLidarData = getLidarData(tf_luna_right, i2c_port0); // Get and process radar data
        printf("right: distance = %5dcm, strength = %5d, temperature = %5d°C\n",
               rightLidarData.distance, rightLidarData.strength, rightLidarData.temperature);
        LidarData rearLidarData = getLidarData(tf_luna_rear, i2c_port0); // Get and process radar data
        printf("rear: distance = %5dcm, strength = %5d, temperature = %5d°C\n",
               rearLidarData.distance, rearLidarData.strength, rearLidarData.temperature);
        LidarData leftLidarData = getLidarData(tf_luna_left, i2c_port0); // Get and process radar data
        printf("left: distance = %5dcm, strength = %5d, temperature = %5d°C\n",
               leftLidarData.distance, leftLidarData.strength, leftLidarData.temperature);

        sleep_ms(500); // Delay for 1 second
        if (shouldNavigate) {
            // Call the navigate function in the interrupt handler
            navigator->navigate();
            shouldNavigate = false;
        }
    }
}
