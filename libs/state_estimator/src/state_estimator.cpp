//
// Created by robbe on 03/12/2023.
//
#include <cstdio>
#include "state_estimator.h"
#include "drivetrain_config.h"
#include "encoder.hpp"
#include "bno080.h"
#include "tf_luna.h"
namespace STATE_ESTIMATOR {
    StateEstimator *StateEstimator::instancePtr = nullptr;

    StateEstimator::StateEstimator(BNO08x* IMUinstance, i2c_inst_t* port, CONFIG::SteeringStyle direction) : encoders{
            [MOTOR_POSITION::FRONT_LEFT] =new Encoder(pio0, 0, motor2040::ENCODER_A, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            [MOTOR_POSITION::FRONT_RIGHT] =new Encoder(pio0, 1, motor2040::ENCODER_B, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            [MOTOR_POSITION::REAR_LEFT] = new Encoder(pio0, 2, motor2040::ENCODER_C, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV),
            [MOTOR_POSITION::REAR_RIGHT] = new Encoder(pio0, 3, motor2040::ENCODER_D, PIN_UNUSED, Direction::NORMAL_DIR, CONFIG::COUNTS_PER_REV)
    }, timer(new repeating_timer_t), estimatedState(), previousState(), currentDriveTrainState() {
        encoders[MOTOR_POSITION::FRONT_LEFT]->init();
        encoders[MOTOR_POSITION::FRONT_RIGHT]->init();
        encoders[MOTOR_POSITION::REAR_LEFT]->init();
        encoders[MOTOR_POSITION::REAR_RIGHT]->init();
        i2c_port = port;
        // Initialize the State struct member variables
        estimatedState.odometry.x = 0.0f;
        estimatedState.velocity.x_dot = 0.0f;
        estimatedState.odometry.y = 0.0f;
        estimatedState.velocity.y_dot = 0.0f;
        estimatedState.velocity.velocity = 0.0f;
        estimatedState.odometry.heading = 0.0f;
        estimatedState.velocity.angular_velocity = 0.0f;
        estimatedState.driveTrainState.speeds[MOTOR_POSITION::FRONT_LEFT] = 0.0f;
        estimatedState.driveTrainState.speeds[MOTOR_POSITION::FRONT_RIGHT] = 0.0f;
        estimatedState.driveTrainState.speeds[MOTOR_POSITION::REAR_LEFT] = 0.0f;
        estimatedState.driveTrainState.speeds[MOTOR_POSITION::REAR_RIGHT] = 0.0f;
        estimatedState.driveTrainState.angles.left = 0.0f;
        estimatedState.driveTrainState.angles.right = 0.0f;
        estimatedState.tofDistances.front = getLidarData(tf_luna_front, i2c_port).distance;
        estimatedState.tofDistances.right = getLidarData(tf_luna_right, i2c_port).distance;
        estimatedState.tofDistances.rear = getLidarData(tf_luna_rear, i2c_port).distance;
        estimatedState.tofDistances.left = getLidarData(tf_luna_left, i2c_port).distance;
        IMU = IMUinstance;
        
        instancePtr = this;

        if (initialise_heading_offset() == false) {
            while (1){
                printf("failed to set initial heading offset\n");
                sleep_ms(1000);
            }
        }
        
        driveDirection = direction;
        
        setupTimer();
    }

    void StateEstimator::showValues() const {
        //printf("FRONT_LEFT: %ld ", encoders.FRONT_LEFT->count());
        //printf("FRONT_RIGHT: %ld ", encoders.FRONT_RIGHT->count());
        //printf("REAR_LEFT: %ld ", encoders.REAR_LEFT->count());
        //printf("REAR_RIGHT: %ld ", encoders.REAR_RIGHT->count());
        //printf("\n");
        printf("X: %f, Y: %f, Velocity: %f, Heading: %f, turn rate: %f, front ToF: %f\n", 
           estimatedState.odometry.x,
           estimatedState.odometry.y,
           estimatedState.velocity.velocity,
           estimatedState.odometry.heading,
           estimatedState.velocity.angular_velocity,
           estimatedState.tofDistances.front);
    }

    void StateEstimator::showValuesViaCSV() const {
        printf("%.3f, %.3f, %.3f, %.1f\n", 
           estimatedState.odometry.x,
           estimatedState.odometry.y,
           estimatedState.odometry.heading,
           estimatedState.tofDistances.front);
    }

    void StateEstimator::publishState() const {
        showValuesViaCSV();
    }

    void StateEstimator::addObserver(Observer* observer) {
        if (observerCount < 10) {
            observers[observerCount++] = observer;
        }
    }

    void StateEstimator::notifyObservers(const DriveTrainState newState) {
        for (int i = 0; i < observerCount; i++) {
            observers[i]->update(newState);
        }
    }

    void StateEstimator::capture_encoders(Encoder::Capture* encoderCaptures) const {
        for(int i = 0; i < MOTOR_POSITION::MOTOR_POSITION_COUNT; i++) {
            encoderCaptures[i] = encoders[i]->capture();
        }
    }

    float StateEstimator::wrap_pi(const float heading) {
        //constrain heading to +/-pi

        const double wrapped = heading > M_PI ? heading - M_TWOPI : heading < -M_PI ? heading + M_TWOPI : heading;
        return static_cast<float>(wrapped);
    }

    void StateEstimator::calculate_bilateral_speeds(const MotorSpeeds& motor_speeds, const SteeringAngles steering_angles, float& left_speed, float& right_speed) {
        left_speed = (motor_speeds[MOTOR_POSITION::FRONT_LEFT] * cos(steering_angles.left)
                      + motor_speeds[MOTOR_POSITION::REAR_LEFT]) / 2;

        // convert average wheel rotation speed to linear speed
        left_speed = left_speed * CONFIG::WHEEL_DIAMETER / 2;

        right_speed = (motor_speeds[MOTOR_POSITION::FRONT_RIGHT] * cos(steering_angles.right)
                       + motor_speeds[MOTOR_POSITION::REAR_RIGHT]) / 2;
        right_speed = right_speed * CONFIG::WHEEL_DIAMETER / 2;
    }

    void StateEstimator::get_position_delta(Encoder::Capture encoderCaptures[4], float& distance_travelled) const {
        // Calculate average wheel rotation delta for left and right sides
        // for the front wheels we only use the forward component of the movement
        //this should give a more accurate estimate for distance_travelled
        float left_travel = (encoderCaptures[MOTOR_POSITION::FRONT_LEFT].radians_delta() * cos(estimatedState.driveTrainState.angles.left)
                             + encoderCaptures[MOTOR_POSITION::REAR_LEFT].radians_delta()) / 2;
        float right_travel = (encoderCaptures[MOTOR_POSITION::FRONT_RIGHT].radians_delta() * cos(estimatedState.driveTrainState.angles.right)
                              + encoderCaptures[MOTOR_POSITION::REAR_RIGHT].radians_delta()) / 2;

        // convert wheel rotation to distance travelled in meters
        distance_travelled = ((left_travel - right_travel) / 2) * CONFIG::WHEEL_DIAMETER / 2;
    }

    void StateEstimator::calculate_new_position(State& tmpState, const float distance_travelled, const float heading) {
        //use the latest heading and distance travleled to update the estiamted position
        tmpState.odometry.x -= driveDirection * distance_travelled * sin(heading);
        tmpState.odometry.y += driveDirection * distance_travelled * cos(heading);

        //now actually update odometry's heading
        tmpState.odometry.heading = heading;

        //constrain heading to +/-pi
        tmpState.odometry.heading = wrap_pi(tmpState.odometry.heading);
    }

    Velocity StateEstimator::calculate_velocities(const float new_heading, const float previous_heading, const float left_speed, const float right_speed) {
        // TODO return a velocities struct instead of setting individual values
        Velocity tmpVelocity{};
        tmpVelocity.velocity = (left_speed - right_speed) / 2;
        tmpVelocity.x_dot = -driveDirection * tmpVelocity.velocity * sin(new_heading);
        tmpVelocity.y_dot = driveDirection * tmpVelocity.velocity * cos(new_heading);
       
        tmpVelocity.angular_velocity = 1000 * (wrap_pi(new_heading - previous_heading)) / timerInterval;
        return tmpVelocity;
    }

    MotorSpeeds StateEstimator::get_wheel_speeds(const Encoder::Capture* encoderCaptures) {
        MotorSpeeds wheelSpeeds{};
        for(int i = 0; i < MOTOR_POSITION::MOTOR_POSITION_COUNT; i++) {
            wheelSpeeds.speeds[static_cast<MOTOR_POSITION::MotorPosition>(i)] = encoderCaptures[i].radians_per_second();
        }
        return wheelSpeeds;
    }

    SteeringAngles StateEstimator::estimate_steering_angles() const {
        return currentSteeringAngles;
    }

    void StateEstimator::estimateState() {
        // instantiate a copy of the current state
        State tmpState = estimatedState;
        
        //get current encoder state
        Encoder::Capture encoderCaptures[MOTOR_POSITION::MOTOR_POSITION_COUNT];
        capture_encoders(encoderCaptures);

        // calculate position deltas

        float distance_travelled = 0.0f;
        get_position_delta(encoderCaptures, distance_travelled);


        float heading = 0.0f;
        get_latest_heading(heading);

        //calculate new position and orientation
        calculate_new_position(tmpState, distance_travelled, heading);

        //calculate speeds

        //get wheel speeds
        tmpState.driveTrainState.speeds = get_wheel_speeds(encoderCaptures);

        // estimate steering angles
        tmpState.driveTrainState.angles = estimate_steering_angles();

        // calculate left and right speeds
        float left_speed;
        float right_speed;
        calculate_bilateral_speeds(tmpState.driveTrainState.speeds, tmpState.driveTrainState.angles, left_speed, right_speed);

        //calc all velocities
        tmpState.velocity = calculate_velocities(tmpState.odometry.heading, previousState.odometry.heading, left_speed, right_speed);

        // get ToF data
        tmpState.tofDistances.front = getLidarData(tf_luna_front, i2c_port).distance;
        tmpState.tofDistances.right = getLidarData(tf_luna_right, i2c_port).distance;
        tmpState.tofDistances.rear = getLidarData(tf_luna_rear, i2c_port).distance;
        tmpState.tofDistances.left = getLidarData(tf_luna_left, i2c_port).distance;

        // update the estimated states
        previousState = estimatedState;
        estimatedState = tmpState;

        // notify observers of the new state
        notifyObservers(estimatedState.driveTrainState);

    }

    void StateEstimator::get_latest_heading(float& heading) {
      //default latest heading is the current heading
      heading = estimatedState.odometry.heading;
      
      //if possible, update the heading with the latest from the IMU
        if (IMU->getSensorEvent() == true) {
            if (IMU->getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
                heading = IMU->getYaw() - heading_offset;
            }
        }
    }

    void StateEstimator::setupTimer() const {
        // Example configuration (adjust as needed)

        // Set up the repeating timer with the callback
        if (!add_repeating_timer_ms(timerInterval,
                                    reinterpret_cast<repeating_timer_callback_t>(&StateEstimator::timerCallback),
                                    nullptr, timer)) {
            // Handle error if timer creation fails
        }
    }

    void StateEstimator::timerCallback(repeating_timer_t *timer) {
        if (instancePtr != nullptr) {
            instancePtr->estimateState();
            instancePtr->publishState();
        }
    }

    void StateEstimator::updateCurrentSteeringAngles(const SteeringAngles& newSteeringAngles) {
        currentSteeringAngles = newSteeringAngles;
    }

    bool StateEstimator::initialise_heading_offset() {
        // function sets the heading_offset to the current heading
        // returns true if the offset is set, false if timed out (no heading updates available)
        long timeoutDuration = 5000;
        long startTime = millis();
        bool isUpdated = false; // Flag to indicate if heading_offset is updated

        while (millis() - startTime < timeoutDuration && !isUpdated) {
            if (IMU->getSensorEvent() == true) {
                if (IMU->getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
                    heading_offset = IMU->getYaw();
                    isUpdated = true;
                }
            }
        }
        //if the timer expired before the heading was set, return false
        return isUpdated;
    }

    StateEstimator::~StateEstimator() {
        delete encoders[MOTOR_POSITION::FRONT_LEFT];
        delete encoders[MOTOR_POSITION::FRONT_RIGHT];
        delete encoders[MOTOR_POSITION::REAR_LEFT];
        delete encoders[MOTOR_POSITION::REAR_RIGHT];
        // Cancel the timer in the destructor
        cancel_repeating_timer(timer);

    }
}
// STATE_ESTIMATOR

