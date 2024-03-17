#include <cstdio>
#include "navigator.h"
#include "statemanager.h"
#include "state_estimator.h"
#include "drivetrain_config.h"
#include "types.h"

Navigator::Navigator(const Receiver* receiver,
                     STATEMANAGER::StateManager* stateManager,
                     STATE_ESTIMATOR::StateEstimator* stateEstimator,
                     CONFIG::SteeringStyle direction) {
    this->receiver = receiver;
    this->pStateManager = stateManager;
    this->pStateEstimator = stateEstimator;
    driveDirection = direction;
}

void Navigator::navigate() {
    if (receiver->get_receiver_data()) {
        //printf("Receiver data available\n");
        ReceiverChannelValues values = receiver->get_channel_values();
        //printf("AIL: %f ", values.AIL);
        //printf("ELE: %f ", values.ELE);
        //printf("THR: %f ", values.THR);
        //printf("RUD: %f ", values.RUD);
        //printf("AUX: %f ", values.AUX);
        //printf("NC: %f ", values.NC);
        //printf("\n");

        //check if the extra Tx channels should trigger anything
        parseTxSignals(values);

        // send the receiver data to the state manager
        // TODO: use a queue to send the receiver data to the state manager
        VehicleState requestedState{};
        requestedState.velocity.velocity = driveDirection * values.ELE * CONFIG::MAX_VELOCITY;
        requestedState.velocity.angular_velocity = values.AIL * CONFIG::MAX_ANGULAR_VELOCITY;
        pStateManager->requestState(requestedState);
    } else {
        printf("No receiver data available\n");
    }
}

bool Navigator::shouldSetHeading(float signal){
    return (signal < setHeadingThreshold);
}

bool Navigator::shouldSetOdometryOrigin(float signal){
    return (signal > setOriginThreshold);
}

void Navigator::setHeading(){
    pStateEstimator->zero_heading();
}

void Navigator::setOrigin(){
    pStateEstimator->request_odometry_offset(current_state.odometry.x, current_state.odometry.y, 0);
}

void Navigator::update(const COMMON::VehicleState newState) {
    current_state = newState;
}

void Navigator::parseTxSignals(const ReceiverChannelValues& signals){
    // function to use "spare" transmitter channels as auxiliary inputs
    // currently can set (zero) odoemtry heading and and origin
        if (shouldSetHeading(signals.RUD)){
            printf("setting current heading to 0.\n");
            setHeading();
        }
        if (shouldSetOdometryOrigin(signals.RUD)){
            printf("setting current position as zero for odometry.\n");
            setOrigin();
        }
}

Navigator::~Navigator() = default;
