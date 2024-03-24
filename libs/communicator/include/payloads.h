#pragma once

namespace PAYLOADS {
    struct __attribute__((packed)) Payload {
        uint8_t packetType;
        explicit Payload(uint8_t type) : packetType(type) {}
    };

    struct __attribute__((packed)) OutgoingDoubleInt : Payload {
        uint8_t val1;
        uint8_t val2;
        explicit OutgoingDoubleInt(uint8_t val1 = 0, uint8_t val2 = 0) : Payload(0x01), val1(val1), val2(val2) {} // 0x01 is an example value
    };

    struct __attribute__((packed)) OutgoingIntFloat : Payload {
        uint8_t val1;
        float float1;
        explicit OutgoingIntFloat(uint8_t val1 = 0, float float2 = 0.0f) : Payload(0x02), val1(val1), float1(float2) {} // 0x02 is an example value
    };

    struct __attribute__((packed)) IncomingSingleInt : Payload {
        uint8_t val1;
        explicit IncomingSingleInt(uint8_t val1) : Payload(0x03), val1(val1) {} // 0x03 is an example value
    };

    // Payload for requested state
    struct __attribute__((packed)) StatePayload : Payload {
        float velocity;
        float angular_velocity;
        explicit StatePayload(float velocity = 0.0f, float angular_velocity = 0.0f) : Payload(0x04), velocity(velocity), angular_velocity(angular_velocity) {} // 0x04 is an example value
    };

   // payload for seriant transferAvailableStatus
    struct __attribute__((packed)) SerialTransferAvailableStatus : Payload {
        uint8_t available;
        explicit SerialTransferAvailableStatus(bool available = false) : Payload(0x05), available(available) {} // 0x05 is an example value
    };

    // payload for EstimatedState
    struct __attribute__((packed)) EstimatedStatePayload : Payload {
        float x;
        float y;
        float heading;
        float tofFront;
        float tofRear;
        float tofLeft;
        float tofRight;

        explicit EstimatedStatePayload(const COMMON::Pose& odometry,
                                       const COMMON::FourToFDistances& tofDistances) : Payload(0x06),
            x(odometry.x), y(odometry.y), heading(odometry.heading), tofFront(tofDistances.front),
            tofRear(tofDistances.rear), tofLeft(tofDistances.left), tofRight(tofDistances.right) {
        } // 0x06 is an example value
    };
}
