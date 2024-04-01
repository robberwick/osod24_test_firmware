#pragma once

#include "types.h"

namespace PAYLOADS {
    namespace PACKET_TYPE {
        enum PacketType: uint8_t {
            SERIAL_TRANSFER_AVAILABLE_STATUS = 0x01,
            REQUESTED_STATE = 0x02,
            ESTIMATED_STATE = 0x03,
            CELL_STATUS = 0x04,
        };
    }

    // Base class for all payloads
    struct __attribute__((packed)) Payload {
        uint8_t packetType;

        explicit Payload(const PACKET_TYPE::PacketType type) : packetType(static_cast<uint8_t>(type)) {
        }
    };

    // Incoming SerialTransferAvailableStatus payload
    struct __attribute__((packed)) SerialTransferAvailableStatus : Payload {
        uint8_t available;

        explicit SerialTransferAvailableStatus(const bool available = false) : Payload(PACKET_TYPE::SERIAL_TRANSFER_AVAILABLE_STATUS), available(available) {
        }
    };

    // payload for RequestedState
    struct __attribute__((packed)) StatePayload : Payload {
        float velocity;
        float angular_velocity;

        explicit StatePayload(const float velocity = 0.0f, const float angular_velocity = 0.0f) : Payload(PACKET_TYPE::REQUESTED_STATE),
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
            : Payload(PACKET_TYPE::ESTIMATED_STATE),
              timestamp(timestamp),
              x(odometry.x), y(odometry.y), heading(odometry.heading),
              tofFront(tofDistances.front),
              tofRear(tofDistances.rear), tofLeft(tofDistances.left),
              tofRight(tofDistances.right) {
        }
    };

    // payload for CellStatus
    struct __attribute__((packed)) CellStatusPayload : Payload {
        float cell1;
        float cell2;
        float cell3;
        float psu;
        bool allOk;
        bool outOfBalance;
        bool lowCellVoltage;
        bool highCellVoltage;
        bool psuUnderVoltage;

        explicit CellStatusPayload(
            const COMMON::CellStatus& cellStatus)
            : Payload(PACKET_TYPE::CELL_STATUS),
              cell1(cellStatus.voltages.cell1),
              cell2(cellStatus.voltages.cell2),
              cell3(cellStatus.voltages.cell3),
              psu(cellStatus.voltages.psu),
              allOk(cellStatus.allOk),
              outOfBalance(cellStatus.outOfBalance),
              lowCellVoltage(cellStatus.lowCellVoltage),
              highCellVoltage(cellStatus.highCellVoltage),
              psuUnderVoltage(cellStatus.psuUnderVoltage) {
        }
    };
}
