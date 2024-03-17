
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "types.h"

#pragma once

// Radar module communication address
const uint8_t tf_luna_default_address = 0x10;
const uint8_t tf_luna_rear = 0x11;
const uint8_t tf_luna_front = 0x12;
const uint8_t tf_luna_right = 0x13;
const uint8_t tf_luna_left = 0x14;

// Data fetch instruction
const uint8_t getLidarDataCmd[] = {0x5A, 0x05, 0x00, 0x01, 0x60};

struct LidarData {
    int distance;
    int strength;
    int temperature;
};

// Function to get Lidar data
LidarData getSingleLidarData(uint8_t i2c_addr, i2c_inst_t* i2c_port);
COMMON::FourToFDistances getAllLidarDistances(i2c_inst_t* i2c_port);
float convertAndApplyOffset(int distance_cm, float offset);
