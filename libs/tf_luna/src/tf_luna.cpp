#include "pico/stdlib.h"
#include <array>
#include "hardware/i2c.h"
#include "drivetrain_config.h"
#include "tf_luna.h"
#include "drivetrain_config.h"
#include "utils.h"

// Function to get Lidar data
LidarData getSingleLidarData(uint8_t i2c_addr, i2c_inst_t* i2c_port) {
    uint8_t temp[9] = {0};
    const int retval = i2c_write_timeout_us(i2c_port, i2c_addr, getLidarDataCmd, 5, false, CONFIG::I2C_TIMEOUT_US); // Send command
    if (retval == PICO_ERROR_GENERIC || retval == PICO_ERROR_TIMEOUT) {
        printf("I2C write error for ToF\r\n");
        handleI2CError(i2c_port);;
    }
    i2c_read_timeout_us(i2c_port, i2c_addr, temp, 9, false, CONFIG::I2C_TIMEOUT_US); // Read response
    if (retval == PICO_ERROR_GENERIC || retval == PICO_ERROR_TIMEOUT) {
        printf("I2C read error for ToF\r\n");
        handleI2CError(i2c_port);;
    }

    LidarData data = {0, 0, 0}; // Initialize to zero
    
    if (temp[0] == 0x59 && temp[1] == 0x59) {
        data.distance = temp[2] + temp[3] * 256; // Distance value
        data.strength = temp[4] + temp[5] * 256; // Signal strength
        data.temperature = (temp[6] + temp[7] * 256) / 8 - 256; // Chip temperature
    }

    return data;
}

COMMON::FourToFDistances getAllLidarDistances(i2c_inst_t* i2c_port) {
    // function to get the distances of four ToF sensors in meters
    // Get distance from each sensor, convert from centimeters to meters, and apply the offset
    int front_cm = getSingleLidarData(tf_luna_front, i2c_port).distance;
    int right_cm = getSingleLidarData(tf_luna_right, i2c_port).distance;
    int rear_cm = getSingleLidarData(tf_luna_rear, i2c_port).distance;
    int left_cm = getSingleLidarData(tf_luna_left, i2c_port).distance;

    // Convert from centimeters to meters and apply offsets
    float front_m = convertAndApplyOffset(front_cm, CONFIG::TOF_FRONT_OFFSET);
    float right_m = convertAndApplyOffset(right_cm, CONFIG::TOF_RIGHT_OFFSET);
    float rear_m = convertAndApplyOffset(rear_cm, CONFIG::TOF_REAR_OFFSET);
    float left_m = convertAndApplyOffset(left_cm, CONFIG::TOF_LEFT_OFFSET);

    // Return the struct populated with the distances
    return {front_m, right_m, rear_m, left_m};
}

float convertAndApplyOffset(int distance_cm, float offset) {
    // covnerts a sensor reading in cm to metres, and applies an offset. returns a float
    return static_cast<float>(distance_cm) / 100.0f + offset;
}