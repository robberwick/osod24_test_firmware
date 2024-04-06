#include "pico/stdlib.h"
#include <algorithm>
#include "hardware/i2c.h"
#include "balance_port.h"
#include "ads1x15.h"

BalancePort::BalancePort() {
    communicator = &Communicator::getInstance();
}

bool BalancePort::initADC(i2c_inst_t* i2c_port) {
    // Initialize inputVoltagesADC, set gain, etc.

    inputVoltagesADC.setGain(ADSXGain_ONE);

    if (inputVoltagesADC.beginADSX(ADSX_ADDRESS_GND, i2c_port, 100, CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN)) {
      return true;
    } else {
      sleep_ms(5000);
      printf("ADS1x15 : Failed to initialize ADS.!\r\n");
      return false;
    }
}

COMMON::adcVoltages BalancePort::getCellVoltages() {
    COMMON::adcVoltages voltages;
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

COMMON::CellStatus BalancePort::checkVoltages(COMMON::adcVoltages measuredVoltages) {
    COMMON::CellStatus voltageStatus{};
    voltageStatus.voltages =  measuredVoltages;
    float cell1 = measuredVoltages.cell1;
    float cell2 = measuredVoltages.cell2;
    float cell3 = measuredVoltages.cell3;
    float PSU = measuredVoltages.psu;

    float maxVoltage = std::max({cell1, cell2, cell3});
    float minVoltage = std::min({cell1, cell2, cell3});

    // voltage is out of balance if the difference between the max and min voltage is greater than the balance threshold
    voltageStatus.outOfBalance = (maxVoltage - minVoltage) > balanceThreshold;
    // voltage is low if the minimum cell voltage is less than the minimum cell voltage threshold
    voltageStatus.lowCellVoltage = minVoltage < minCellVoltage;
    // voltage is high if the maximum cell voltage is greater than the maximum cell voltage threshold
    voltageStatus.highCellVoltage = maxVoltage > maxCellVoltage;
    // voltage is under voltage if the PSU voltage is greater than the PSU connected threshold and less than the minimum PSU voltage
    voltageStatus.psuUnderVoltage = PSU > PSUConnectedThreshold && PSU < minPSU;
    // voltage is okay if none of the above conditions are met
    voltageStatus.allOk = !voltageStatus.lowCellVoltage && !voltageStatus.highCellVoltage && !voltageStatus.outOfBalance && !voltageStatus.psuUnderVoltage;
    return voltageStatus;
}

void BalancePort::raiseCellStatus() {
    const COMMON::adcVoltages voltages = getCellVoltages(); // Assume this method exists and fetches voltages
    const COMMON::CellStatus status = checkVoltages(voltages);
    if (!status.allOk) {
        // Increment failCount if the cell status is not okay for 10 consecutive times
        failCount++;
        if (failCount > failCountThreshold) {
            PAYLOADS::CellStatusPayload cellStatuspayload(
            status
        );
            communicator->sendPacket(cellStatuspayload);
        }
    } else {
        failCount = 0;
    }
}
