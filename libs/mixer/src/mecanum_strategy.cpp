//
// Created by markm on 11/11/2023.
//
// code for a four wheeled, independent four wheel drive, two wheel steering platform

#include "mecanum_strategy.h"
#include "mixer_strategy.h"
#include <cmath>
#include <limits>
#include <cstdio>
#include <functional>
#include <algorithm>
#include "types.h"

/**
 * @brief Returns the sign of a value
 * @tparam T The type of the value
 * @param value The value to check
 * @return 1 if the value is positive, -1 if the value is negative
 */
template<typename T>
int sign(T value) {
    return std::signbit(value) ? -1 : 1;
}

namespace MIXER {
    MecanumMixer::MecanumMixer(float track, float base, float angle) : wheelTrack(track), wheelBase(base),
                                                                           maxSteeringAngle(angle) {
        // initialise the turn radius to zero
        turnRadius = 0.0;
    }

    DriveTrainState MecanumMixer::mix(float velocity, float angularVelocity, float strafeVelocity) {
        // function takes desired forward speed ("throttle") and strafe speed ("rudder") in m/s and
        // turn rate in radians/sec, and outputs individual wheel speeds in m/s
        // and turn angle of steerable wheels in radians (0, or straight ahead)

        float frontLeft = velocity - angularVelocity + strafeVelocity;
        float frontRight = velocity + angularVelocity - strafeVelocity;
        float rearLeft = velocity - angularVelocity - strafeVelocity;
        float rearRight = velocity + angularVelocity + strafeVelocity;

        // Calculating the maximum absolute value to scale the speeds
        float max_abs = std::max(std::max(abs(frontLeft), abs(frontRight)), std::max(abs(rearLeft), abs(rearRight)));

        if (max_abs > 1.0) {
            frontLeft /= max_abs;
            frontRight /= max_abs;
            rearLeft /= max_abs;
            rearRight /= max_abs;
        }

        const float speed_factor = 1.0f;
        float scaled_FL = frontLeft * speed_factor;
        float scaled_FR = frontRight * speed_factor * -1;
        float scaled_RL = rearLeft * speed_factor;
        float scaled_RR = rearRight * speed_factor * -1;

        DriveTrainState result = {};
        result.speeds = {};
        result.speeds[MOTOR_POSITION::FRONT_LEFT] = scaled_FL;
        result.speeds[MOTOR_POSITION::FRONT_RIGHT] = scaled_FR;
        result.speeds[MOTOR_POSITION::REAR_LEFT] = scaled_RL;
        result.speeds[MOTOR_POSITION::REAR_RIGHT] = scaled_RR;
        result.angles = {0, 0};

        return result;

    }

} // namespace MIXER