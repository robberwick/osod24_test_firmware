#include "receiver.h"
#include "statemanager.h"
#include "types.h"
#include "interfaces.h"
#include "drivetrain_config.h"

using namespace COMMON;
class Navigator: public Observer {
public:
    explicit Navigator(const Receiver* receiver, 
                        STATEMANAGER::StateManager *stateManager,
                        STATE_ESTIMATOR::StateEstimator* estimator,
                        CONFIG::SteeringStyle direction);
    ~Navigator();
    void navigate();
    CONFIG::SteeringStyle driveDirection; //factor to change requested motor speed direction based on what we currently consider the front
    void update(const VehicleState newState) override;

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    STATE_ESTIMATOR::StateEstimator* pStateEstimator;
    VehicleState current_state;
    float setHeadingThreshold = -0.5; //if signal below this, set the heading
    float setOriginThreshold = 0.5; //if signal above this, set the odometry origin
    bool shouldSetHeading(float signal);
    bool shouldSetOdometryOrigin(float signal);
    void setHeading(); //local method that's linked to the stateEstimator set_Heading_Offset method
    void setOrigin(); //local method that's linked to the stateEstimator set_Odometry_Offset method
    void parseTxSignals(ReceiverChannelValues signals); //function to use "spare" transmitter channels as auxiliary inputs
};