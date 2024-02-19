#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "tf_luna.h"

// Function to get Lidar data
LidarData getLidarData(uint8_t i2c_addr) {
    uint8_t temp[9] = {0};
    i2c_write_blocking(i2c1, i2c_addr, getLidarDataCmd, 5, false); // Send command
    i2c_read_blocking(i2c1, i2c_addr, temp, 9, false); // Read response

    LidarData data = {0, 0, 0}; // Initialize to zero

    if (temp[0] == 0x59 && temp[1] == 0x59) {
        data.distance = temp[2] + temp[3] * 256; // Distance value
        data.strength = temp[4] + temp[5] * 256; // Signal strength
        data.temperature = (temp[6] + temp[7] * 256) / 8 - 256; // Chip temperature
    }

    return data;
}

