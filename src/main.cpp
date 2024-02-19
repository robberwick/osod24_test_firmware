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
#include "ads1x15.h"

PICO_ADS1015 ads; // 12 bit ADS1015 instance

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

    int16_t adc0, adc1, adc2, adc3;
    float volts0, volts1, volts2, volts3;
    ads.setGain(ADSXGain_ONE);

    if (!ads.beginADSX(ADSX_ADDRESS_GND, i2c_port0, 100, CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN)) {
      while (1){
        printf("ADS1x15 : Failed to initialize ADS.!\r\n");
        sleep_ms(5000); // Delay for 5 seconds
      };
    }
    // set up the state estimator
    auto *pStateEstimator = new STATE_ESTIMATOR::StateEstimator();

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
        // int16_t batteryVoltages[3] = getCellVoltages(ADS1015_address, i2c_port0);
        adc0 = ads.readADC_SingleEnded(ADSX_AIN0);
        adc1 = ads.readADC_SingleEnded(ADSX_AIN1);
        adc2 = ads.readADC_SingleEnded(ADSX_AIN2);
        adc3 = ads.readADC_SingleEnded(ADSX_AIN3);

        volts0 = 6 * ads.computeVolts(adc0);
        volts1 = 6 * ads.computeVolts(adc1);
        volts2 = 6 * ads.computeVolts(adc2);
        volts3 = 6 * ads.computeVolts(adc3);
    

        printf("cell 1: %fV, cell 2: %fV, cell 3: %fV, PSU: %fV\n",
               volts0-volts1, volts1-volts2, volts2, volts3);
                
        sleep_ms(100); // Delay for 0.5 second
        // Do nothing in the main loop
        if (shouldNavigate) {
            // Call the navigate function in the interrupt handler
            navigator->navigate();
            shouldNavigate = false;
        }
    }
}
