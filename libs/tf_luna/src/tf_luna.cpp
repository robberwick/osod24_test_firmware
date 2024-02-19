#include "pico/stdlib.h"
#include <array>
#include "hardware/i2c.h"
#include "tf_luna.h"

// Function to get Lidar data
LidarData getSingleLidarData(uint8_t i2c_addr, i2c_inst_t* i2c_port) {
    uint8_t temp[9] = {0};
    i2c_write_blocking(i2c_port, i2c_addr, getLidarDataCmd, 5, false); // Send command
    i2c_read_blocking(i2c_port, i2c_addr, temp, 9, false); // Read response

    LidarData data = {0, 0, 0}; // Initialize to zero
    
    if (temp[0] == 0x59 && temp[1] == 0x59) {
        data.distance = temp[2] + temp[3] * 256; // Distance value
        data.strength = temp[4] + temp[5] * 256; // Signal strength
        data.temperature = (temp[6] + temp[7] * 256) / 8 - 256; // Chip temperature
    }

    return data;
}

FourTofDistances getAllLidarDistances(i2c_inst_t* i2c_port) {
    // function to get the distances of four ToF sensors
    // Get distance from each sensor
    int frontDistance = getSingleLidarData(tf_luna_front, i2c_port).distance;
    int rightDistance = getSingleLidarData(tf_luna_right, i2c_port).distance;
    int rearDistance = getSingleLidarData(tf_luna_rear, i2c_port).distance;
    int leftDistance = getSingleLidarData(tf_luna_left, i2c_port).distance;

    // Return the struct populated with the distances
    return {frontDistance, rightDistance, rearDistance, leftDistance};
}
