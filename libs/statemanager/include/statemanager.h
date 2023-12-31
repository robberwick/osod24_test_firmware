//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_STATEMANAGER_H
#define OSOD_MOTOR_2040_STATEMANAGER_H

#include "types.h"
#include "receiver.h"
#include "state_estimator.h"
#include "stoker.h"
#include "mixer_strategy.h"
#include "servo.hpp"

namespace STATEMANAGER {

    struct SteeringServos {
        servo::Servo* left;
        servo::Servo* right;
    };

    struct Stokers {
        STOKER::Stoker* FRONT_LEFT;
        STOKER::Stoker* FRONT_RIGHT;
        STOKER::Stoker* REAR_LEFT;
        STOKER::Stoker* REAR_RIGHT;
    };

    class StateManager {
    public:
        explicit StateManager(MIXER::MixerStrategy *mixerStrategy, STATE_ESTIMATOR::StateEstimator *stateEstimator);

        void requestState(STATE_ESTIMATOR::State requestedState);
    private:
        MIXER::MixerStrategy *mixerStrategy;
        STATE_ESTIMATOR::StateEstimator *stateEstimator;
        Stokers stokers{};
        SteeringServos steering_servos{};
        // max speed factor - scale the speed of the motors down to this value
        static constexpr float SPEED_EXTENT = 1.0f;

        void setSpeeds(COMMON::MixerOutput motorSpeeds) const;
    };

} // StateManager

#endif //OSOD_MOTOR_2040_STATEMANAGER_H
