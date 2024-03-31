#pragma once

#include "types.h"

namespace PAYLOADS {
    // Base class for all payloads
    struct __attribute__((packed)) Payload {
        uint8_t packetType;

        explicit Payload(const uint8_t type) : packetType(type) {
        }
    };

    // Incoming SerialTransferAvailableStatus payload
    struct __attribute__((packed)) SerialTransferAvailableStatus : Payload {
        uint8_t available;

        explicit SerialTransferAvailableStatus(const bool available = false) : Payload(0x01), available(available) {
        }
    };

    // payload for RequestedState
    struct __attribute__((packed)) StatePayload : Payload {
        float velocity;
        float angular_velocity;

        explicit StatePayload(const float velocity = 0.0f, const float angular_velocity = 0.0f) : Payload(0x01),
            velocity(velocity),
            angular_velocity(angular_velocity) {
        }
    };

    // payload for EstimatedState
    struct __attribute__((packed)) EstimatedStatePayload : Payload {
        uint32_t timestamp;
        float x;
        float y;
        float heading;
        float tofFront;
        float tofRear;
        float tofLeft;
        float tofRight;

        explicit EstimatedStatePayload(
            const uint32_t timestamp,
            const COMMON::Pose& odometry,
            const COMMON::FourToFDistances& tofDistances)
            : Payload(0x03),
              timestamp(timestamp),
              x(odometry.x), y(odometry.y), heading(odometry.heading),
              tofFront(tofDistances.front),
              tofRear(tofDistances.rear), tofLeft(tofDistances.left),
              tofRight(tofDistances.right) {
        }
    };
}
