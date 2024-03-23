//
// Created by robbe on 31/12/2023.
//

#ifndef OSOD_MOTOR_2040_TYPES_H
#define OSOD_MOTOR_2040_TYPES_H
#include <cstddef>

#pragma once
namespace COMMON {
    namespace MOTOR_POSITION {
        enum MotorPosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT,
            MOTOR_POSITION_COUNT // always keep this last in the enum so that we can use it to get the number of elements
        };
    }
    struct MotorSpeeds {
        float speeds[MOTOR_POSITION::MOTOR_POSITION_COUNT];

        float& operator[](const MOTOR_POSITION::MotorPosition position) {
            return speeds[position];
        }

        const float& operator[](const MOTOR_POSITION::MotorPosition position) const {
            return speeds[position];
        }
    };

    struct SteeringAngles {
        float left;
        float right;
    };

    struct DriveTrainState {
        MotorSpeeds speeds;
        SteeringAngles angles;
    };

    struct FourToFDistances {
        float front;
        float right;
        float rear;
        float left;
    };

    constexpr size_t NUM_TOF_SENSORS = sizeof(FourToFDistances) / sizeof(float);

    struct Pose {
        float x;
        float y;
        float heading;
    };

    struct Point {
        float x;
        float y;
    };

    struct Waypoint {
        Point position;
        float heading;
        float speed;
    };
    namespace NAVIGATION_MODE {
        enum Mode {
            REMOTE_CONTROL,
            WAYPOINT,
            PI_CONTROL
        };
    }
    struct Velocity {
        float x_dot;
        float y_dot;
        float velocity;
        float angular_velocity;
    };
    struct VehicleState {
        Velocity velocity;
        Pose odometry;
        DriveTrainState driveTrainState;
        FourToFDistances tofDistances;
    };
}

#endif //OSOD_MOTOR_2040_TYPES_H
