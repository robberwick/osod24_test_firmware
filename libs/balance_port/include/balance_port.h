
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ads1x15.h"

// ADC address
const uint8_t ADS1015_address = 0x48;

extern PICO_ADS1015 inputVoltagesADC; // Assuming ads is initialized elsewhere, like in main.cpp

struct adcVoltages {
    float cell1;
    float cell2;
    float cell3;
    float psu;
};

// Function to get Lidar data
adcVoltages getCellVoltages();
