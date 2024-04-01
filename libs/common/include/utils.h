#ifndef UTILS_H
#define UTILS_H
#include <cstdio>
#include <math.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "drivetrain_config.h"

extern volatile bool ESCdelayInProgress;
extern volatile bool ESCirqTriggered;

void initI2C(i2c_inst_t* &i2c_port, uint baudrate, uint sda_pin, uint scl_pin);

float wrap_pi(const float heading);

void initMotorMonitorPins();

void handlerMotorController(uint gpio, uint32_t events);

bool non_blocking_delay_us(uint32_t delay_us);

void toggleMotorSleepPin();

#endif // UTILS_H