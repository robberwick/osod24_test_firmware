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
#include "balance_port.h"
#include "bno080.h"


Navigator *navigator;
int32_t navigationPeriodMs = 20;
bool shouldNavigate = false;
bool shouldReadCellStatus = false;
int32_t navigateCount = 0;

// calcluate the period to read the cell status - divide the time in ms by the navigation period, and floor the result
int32_t shouldReadCellCount = 2000 / navigationPeriodMs;


extern "C" void timer_callback(repeating_timer_t *t) {
    // Call the navigate function in the interrupt handler
    shouldNavigate = true;
    navigateCount++;
    if (navigateCount % shouldReadCellCount == 0) {
        navigateCount = 0;
        shouldReadCellStatus = true;
    }
}

int main() {
    stdio_init_all();
    i2c_inst_t* i2c_port0;
    initI2C(i2c_port0, 100 * 1000, CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN);
bool adcPresent;
    BalancePort balancePort;
    adcPresent = balancePort.initADC(i2c_port0); // Initialize ADC

    //set up IMU
    BNO08x IMU;
    if (IMU.begin(CONFIG::BNO08X_ADDR, i2c_port0)==false) {
        while (1){
            printf("BNO08x not detected at default I2C address. Check wiring. Freezing\n");
            sleep_ms(1000);
        }
    }
    IMU.enableRotationVector();

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
            navigationPeriodMs,
            reinterpret_cast<repeating_timer_callback_t>(timer_callback),
            nullptr,
            &navigationTimer
    );

    while (true) {
        if (shouldNavigate) {
            // Call the navigate function
            navigator->navigate();
            shouldNavigate = false;
        }

        if (adcPresent && shouldReadCellStatus){
            balancePort.raiseCellStatus();
        }
    }
}
