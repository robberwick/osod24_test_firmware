//
// Created by robbe on 03/12/2023.
//

#ifndef OSOD_MOTOR_2040_STATE_ESTIMATOR_H
#define OSOD_MOTOR_2040_STATE_ESTIMATOR_H
#include <vector>
#include <numeric>
#include <cmath>
#include <tuple>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "motor2040.hpp"
#include "drivetrain_config.h"
#include "interfaces.h"
#include "types.h"
#include "bno080.h"
#include "tf_luna.h"

using namespace motor;
using namespace encoder;

struct Encoders {
    Encoder* FRONT_LEFT;
    Encoder* FRONT_RIGHT;
    Encoder* REAR_LEFT;
    Encoder* REAR_RIGHT;
};

namespace STATE_ESTIMATOR {
    using namespace COMMON;
    using namespace std;

    // define a State struct containing the state parameters that can be requested or tracked
    struct State {
        Velocity velocity;
        Pose odometry;
        DriveTrainState driveTrainState;
        FourTofDistances tofDistances;
    };

    class StateEstimator : public Subject {
    public:
        explicit StateEstimator(BNO08x* IMUinstance, i2c_inst_t* port, CONFIG::SteeringStyle direction, float arenaDimension);

    protected:
        ~StateEstimator(); // Destructor to cancel the timer
    public:
        void showValues() const;
        void showValuesViaCSV() const;
        
        void estimateState();

        void publishState() const;

        void addObserver(Observer* observer) override;

        void notifyObservers(DriveTrainState newState) override;

        void updateCurrentSteeringAngles(const SteeringAngles& newSteeringAngles);

        static float wrap_pi(float heading);

        void calculateBilateralSpeeds(const MotorSpeeds& motor_speeds, SteeringAngles steering_angles,
                                        float& left_speed, float& right_speed);

        CONFIG::SteeringStyle driveDirection; //factor to change odometry direction based on what we currently consider the front

        Pose localisation(float heading, FourTofDistances tof_distances);

        std::pair<float, float> possiblePositions(float heading, float distance, float arena_size);

        std::tuple<float, float, float> coordinateVariance(const std::vector<float>& xList, const std::vector<float>& yList);

        bool arenaLocalisation;
        struct PermutationResult {
            std::array<float, NUM_TOF_SENSORS> xList;
            std::array<float, NUM_TOF_SENSORS> yList;
            size_t xSize;
            size_t ySize;
        };

    private:
        Encoder* encoders[MOTOR_POSITION::MOTOR_POSITION_COUNT];
        static StateEstimator* instancePtr;
        repeating_timer_t* timer;
        BNO08x* IMU;
        i2c_inst_t* i2c_port;
        float heading_offset;
        //TODO: (related to issue #42) actually use timer (defined above) instead of fixed interval
        const uint32_t timerInterval = 50;  // Interval in milliseconds
        State estimatedState;
        State previousState;
        DriveTrainState currentDriveTrainState;
        SteeringAngles currentSteeringAngles;
        float arenaSize;
        float localisation_weighting = 0.01;
        Pose localisationEstimate;

        static void timerCallback(repeating_timer_t* timer);

        void setupTimer() const;

        Observer* observers[10] = {};
        int observerCount = 0;

        void captureEncoders(Encoder::Capture* encoderCaptures) const;
        
        void getLatestHeading(float& heading);

        bool initialiseHeadingOffset();

        void getPositionDelta(Encoder::Capture encoderCaptures[4], float& distance_travelled) const;

        void calculateNewPosition(State& tmpState, float distance_travelled, float heading);

        Velocity calculateVelocities(float new_heading, float previous_heading, float left_speed, float right_speed);

        static MotorSpeeds getWheelSpeeds(const Encoder::Capture* encoderCaptures);

        [[nodiscard]] SteeringAngles estimateSteeringAngles() const;
        
        pair<float, float> calculatePossiblePositions(float heading, float distance);

        tuple<float, float, float> calculateCoordinateVariance(const PermutationResult& result);

        Pose filterPositions(Pose odometryEstimate, Pose localisationEstimate);

        PermutationResult createPermutation(
            int permutation,
            const std::array<float, NUM_TOF_SENSORS>& xPositions,
            const std::array<float, NUM_TOF_SENSORS>& yPositions);

    };
} // STATE_ESTIMATOR

#endif //OSOD_MOTOR_2040_STATE_ESTIMATOR_H
