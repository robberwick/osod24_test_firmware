#include <cstdio>
#include "navigator.h"
#include "statemanager.h"
#include "state_estimator.h"
#include "types.h"
#include "drivetrain_config.h"

Navigator::Navigator(const Receiver* receiver,
                     STATEMANAGER::StateManager* stateManager,
                     STATE_ESTIMATOR::StateEstimator* stateEstimator) {
    this->receiver = receiver;
    this->pStateManager = stateManager;
    this->pStateEstimator = stateEstimator;
}

void Navigator::navigate() {
    //printf("Navigating...\n");
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

        // send the receiver data to the state manager
        // TODO: use a queue to send the receiver data to the state manager
        VehicleState requestedState{};
        requestedState.velocity.velocity = values.ELE * CONFIG::MAX_VELOCITY;
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
    //#TODO: add a timeout blocker to prevent this function being called twice in quick succession?
    pStateEstimator->set_heading_offset();
}

void Navigator::setOrigin(){
    //As above: #TODO: add a timeout blocker to prevent this function being called twice in quick succession
    // as that undos the original reset, if its called before the values have propogated through
    pStateEstimator->apply_odometry_offset(current_state.odometry.x, current_state.odometry.y);
}

void Navigator::update(const COMMON::VehicleState newState) {
    current_state = newState;
}

void Navigator::parseTxSignals(ReceiverChannelValues signals){
    // function to use "spare" transmitter channels as auxiliary inputs
    // currently can set (zero) odoemtry heading and and origin
        if (shouldSetHeading(signals.RUD)){
            printf("setting current heading to 0.\n");
            setHeading();
            //a time delay seems to be needed after setting the heading to allow
            //the signals to propogate through before this function can be called again.
            sleep_ms(100);
        }
        if (shouldSetOdometryOrigin(signals.RUD)){
            printf("setting current position as zero for odometry.\n");
            setOrigin();
            sleep_ms(100);
        }
}

Navigator::~Navigator() = default;
