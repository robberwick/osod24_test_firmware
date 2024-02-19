
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ads1x15.h"
#include "drivetrain_config.h"

// ADC address
const uint8_t ADS1015_address = 0x48;

struct adcVoltages {
    float cell1;
    float cell2;
    float cell3;
    float psu;
};

class BalancePort {
public:
    BalancePort(); // Constructor for initializing the ADC
    void initADC(i2c_inst_t* i2c_port); // Method to initialize ADC settings
    adcVoltages getCellVoltages(); // Method to read and return cell voltages

private:
    PICO_ADS1015 inputVoltagesADC; // ADS1015 object
};