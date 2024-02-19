
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


// Radar module communication address
const uint8_t tf_luna_default_address = 0x10;

// Data fetch instruction
const uint8_t getLidarDataCmd[] = {0x5A, 0x05, 0x00, 0x01, 0x60};

struct LidarData {
    int distance;
    int strength;
    int temperature;
};

// Function to initialize I2C
void initI2C();

// Function to get Lidar data
LidarData getLidarData(uint8_t i2c_addr);
