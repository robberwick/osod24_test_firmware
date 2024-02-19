#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "balance_port.h"
#include "ads1x15.h"

void BalancePort::initADC(i2c_inst_t* i2c_port) {
    // Initialize inputVoltagesADC, set gain, etc.

    inputVoltagesADC.setGain(ADSXGain_ONE);

    if (!inputVoltagesADC.beginADSX(ADSX_ADDRESS_GND, i2c_port, 100, CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN)) {
      while (1){
        printf("ADS1x15 : Failed to initialize ADS.!\r\n");
        sleep_ms(5000); // Delay for 5 seconds
      };
    }
}

adcVoltages BalancePort::getCellVoltages() {
    adcVoltages voltages;
    int16_t adc0, adc1, adc2, adc3;

    adc0 = inputVoltagesADC.readADC_SingleEnded(ADSX_AIN0);
    adc1 = inputVoltagesADC.readADC_SingleEnded(ADSX_AIN1);
    adc2 = inputVoltagesADC.readADC_SingleEnded(ADSX_AIN2);
    adc3 = inputVoltagesADC.readADC_SingleEnded(ADSX_AIN3);

    voltages.cell1 = 6 * (inputVoltagesADC.computeVolts(adc0) - inputVoltagesADC.computeVolts(adc1));
    voltages.cell2 = 6 * (inputVoltagesADC.computeVolts(adc1) - inputVoltagesADC.computeVolts(adc2));
    voltages.cell3 = 6 * inputVoltagesADC.computeVolts(adc2);
    voltages.psu = 6 * inputVoltagesADC.computeVolts(adc3);

    return voltages;
}

