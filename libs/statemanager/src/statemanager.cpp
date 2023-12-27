//
// Created by robbe on 02/12/2023.
//

#include <cstdio>
#include "statemanager.h"
#include "motor2040.hpp"
#include "servo.hpp"

namespace STATEMANAGER {

    StateManager::StateManager(MIXER::MixerStrategy *mixerStrategy) : mixerStrategy(mixerStrategy) {
        printf("State manager created\n");
        // set up the stokers
        stokers.FRONT_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_A, Direction::NORMAL_DIR);
        stokers.FRONT_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_B, Direction::NORMAL_DIR);
        stokers.REAR_LEFT = new STOKER::Stoker(motor::motor2040::MOTOR_C, Direction::NORMAL_DIR);
        stokers.REAR_RIGHT = new STOKER::Stoker(motor::motor2040::MOTOR_D, Direction::NORMAL_DIR);
        steering_servos.left = new servo::Servo(28); //ADC2 / PWM 6
        steering_servos.right = new servo::Servo(16); //PWM 0
        steering_servos.left->init();
        steering_servos.right->init();
        steering_servos.left->calibration().apply_three_pairs(2200, 1599, 1032, -0.7854, 0, 0.7854);
        steering_servos.left->calibration().apply_three_pairs(2200, 1670, 1221, -0.7854, 0, 0.7854);
        steering_servos.left->enable();
        steering_servos.right->enable();
        steering_servos.left->to_mid();
        steering_servos.right->to_mid();

    };

    void StateManager::requestState(RequestedState requestedState) {
        //printf("Requested state...\n");
        //printf("Velocity: %f ", requestedState.velocity);
        //printf("Angular velocity: %f ", requestedState.angularVelocity);
        //printf("\n");
        MIXER::AckermannOutput ackermannOutput = mixerStrategy->mix(requestedState.velocity, requestedState.angularVelocity);
        setSpeeds(ackermannOutput);
    }

    void StateManager::setSpeeds(MIXER::AckermannOutput ackermannOutput) const {
        stokers.FRONT_LEFT->set_speed(ackermannOutput.frontLeftSpeed);
        stokers.FRONT_RIGHT->set_speed(ackermannOutput.frontRightSpeed);
        stokers.REAR_LEFT->set_speed(ackermannOutput.rearLeftSpeed);
        stokers.REAR_RIGHT->set_speed(ackermannOutput.rearRightSpeed);
        if (std::fabs(ackermannOutput.frontLeftSpeed) > 0.02) {
            steering_servos.left->value(ackermannOutput.frontLeftAngle);
        }
        if (std::fabs(ackermannOutput.frontRightSpeed) > 0.02) {
            steering_servos.right->value(ackermannOutput.frontRightAngle);
        }
    }
} // StateManager