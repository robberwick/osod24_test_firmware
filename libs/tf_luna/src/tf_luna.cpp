#include "pico/stdlib.h"
#include <array>
#include "hardware/i2c.h"
#include "tf_luna.h"

// Function to get Lidar data
LidarData getLidarData(uint8_t i2c_addr, i2c_inst_t* i2c_port) {
    std::array<uint8_t, 9> temp = {0};
    i2c_write_blocking(i2c_port, i2c_addr, getLidarDataCmd, 5, false); // Send command
    i2c_read_blocking(i2c_port, i2c_addr, temp.data(), 9, false); // Read response

    LidarData data = {0, 0, 0}; // Initialize to zero

    auto [header1, header2, distLow, distHigh, strengthLow, strengthHigh, tempLow, tempHigh, _] = temp;
    
    if (header1 == 0x59 && header2 == 0x59) {
        data.distance = distLow + distHigh * 256; // Distance value
        data.strength = strengthLow + strengthHigh * 256; // Signal strength
        data.temperature = (tempLow + tempHigh * 256) / 8 - 256; // Chip temperature
    }

    return data;
}

FourTofDistances getAllLidarDistances(i2c_inst_t* i2c_port) {
    // function to get the distances of four ToF sensors
    // Get distance from each sensor
    int frontDistance = getLidarData(tf_luna_front, i2c_port).distance;
    int rightDistance = getLidarData(tf_luna_right, i2c_port).distance;
    int rearDistance = getLidarData(tf_luna_rear, i2c_port).distance;
    int leftDistance = getLidarData(tf_luna_left, i2c_port).distance;

    // Return the struct populated with the distances
    return {frontDistance, rightDistance, rearDistance, leftDistance};
}
