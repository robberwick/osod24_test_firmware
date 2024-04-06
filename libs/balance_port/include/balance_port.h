
#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ads1x15/ads1x15.hpp"
#include "drivetrain_config.h"
#include "types.h"
#include "communicator.h"

// ADC address
const uint8_t ADS1015_address = 0x48;



class BalancePort {
public:
    BalancePort(); // Constructor for initializing the ADC
    bool initADC(i2c_inst_t* i2c_port); // Method to initialize ADC settings
    COMMON::CellStatus checkVoltages(COMMON::adcVoltages measuredVoltages);
    COMMON::adcVoltages getCellVoltages(); // Method to read and return cell voltages
    void raiseCellStatus();
    float minCellVoltage = 3.5;
    float maxCellVoltage = 4.2;
    float balanceThreshold = 0.2;
    float minPSU = 11;
    float PSUConnectedThreshold = 5;
    int failCountThreshold = 5;

private:
    PICO_ADS1015 inputVoltagesADC; // ADS1015 object
    int failCount = 0;
    Communicator* communicator;
};