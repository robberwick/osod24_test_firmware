#include <cstdio>
#include "pico/stdlib.h"
#include "navigator.h"
#include "receiver.h"
#include "statemanager.h"
#include "state_estimator.h"
#include "motor2040.hpp"
#include "tank_steer_strategy.h"
#include "drivetrain_config.h"

int main() {
    stdio_init_all();

    // set up the state estimator
    auto* pStateEstimator = new STATE_ESTIMATOR::StateEstimator();

    // set up the state manager
    using namespace STATEMANAGER;
    auto* pTankSteerStrategy = new MIXER::TankSteerStrategy(CONFIG::WHEEL_TRACK, CONFIG::WHEEL_BASE);
    auto* pStateManager = new StateManager(pTankSteerStrategy);

    // set up the receiver
    // if the cmake build flag RX_PROTOCOL is CPPM, then use the CPPM receiver
    // otherwise use the SBUS receiver
    Receiver* pReceiver = getReceiver(motor::motor2040::RX_ECHO);

    // set up the navigator
    Navigator navigator = Navigator(pReceiver, pStateManager);

    while (true) {
        navigator.navigate();
        sleep_ms(20);
    }
}
