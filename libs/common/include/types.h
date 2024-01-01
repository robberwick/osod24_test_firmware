//
// Created by robbe on 31/12/2023.
//

#ifndef OSOD_MOTOR_2040_TYPES_H
#define OSOD_MOTOR_2040_TYPES_H

namespace COMMON {
    struct MotorSpeeds {
        float frontLeft;
        float frontRight;
        float rearLeft;
        float rearRight;
    };

    struct SteeringAngles {
        float left;
        float right;
    };

    struct DriveTrainState {
        MotorSpeeds speeds;
        SteeringAngles angles;
    };
}

#endif //OSOD_MOTOR_2040_TYPES_H
