
#include <stdio.h>
#include <string>
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
struct CellStatus {
    adcVoltages voltages; // Include this only if needed
    bool allOk; // True if all cells are within the acceptable range
    bool outOfBalance;
    bool lowCellVoltage;
    bool highCellVoltage;
    bool psuUnderVoltage;
    std::string fault;
};

class BalancePort {
public:
    //BalancePort(); // Constructor for initializing the ADC
    bool initADC(i2c_inst_t* i2c_port); // Method to initialize ADC settings
    CellStatus checkVoltages(adcVoltages measuredVoltages);
    adcVoltages getCellVoltages(); // Method to read and return cell voltages
    CellStatus getCellStatus();
    float minCellVoltage = 3.5;
    float maxCellVoltage = 4.2;
    float balanceThreshold = 0.2;
    float minPSU = 11;
    float PSUConnectedThreshold = 5;
private:
    PICO_ADS1015 inputVoltagesADC; // ADS1015 object
};