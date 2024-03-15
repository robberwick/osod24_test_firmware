#include "pico/stdlib.h"
#include <array>
#include "hardware/i2c.h"
#include "tf_luna.h"
#include "drivetrain_config.h"

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
    // function to get the distances of four ToF sensors in meters
    // Get distance from each sensor, convert from millimeters to meters, and apply the offset
    int front_mm = getSingleLidarData(tf_luna_front, i2c_port).distance;
    int right_mm = getSingleLidarData(tf_luna_right, i2c_port).distance;
    int rear_mm = getSingleLidarData(tf_luna_rear, i2c_port).distance;
    int left_mm = getSingleLidarData(tf_luna_left, i2c_port).distance;

    // Convert from millimeters to meters and apply offsets
    float front_m = static_cast<float>(front_mm) / 1000.0f + CONFIG::TOF_FRONT_OFFSET;
    float right_m = static_cast<float>(right_mm ) / 1000.0f + CONFIG::TOF_RIGHT_OFFSET;
    float rear_m = static_cast<float>(rear_mm) / 1000.0f + CONFIG::TOF_REAR_OFFSET;
    float left_m = static_cast<float>(left_mm) / 1000.0f + CONFIG::TOF_LEFT_OFFSET;

    // Return the struct populated with the distances
    return {front_m, right_m, rear_m, left_m};
}
