#include "receiver.h"
#include "statemanager.h"
#include "types.h"
#include "interfaces.h"
#include "drivetrain_config.h"
#include "waypoint_navigation.h"

using namespace COMMON;
class Navigator: public Observer {
public:
    explicit Navigator(const Receiver* receiver, 
                        STATEMANAGER::StateManager *stateManager,
                        STATE_ESTIMATOR::StateEstimator* estimator,
                        CONFIG::DrivingDirection direction);
    ~Navigator();
    void navigate();
    NAVIGATION_MODE::Mode navigationMode;
    void update(VehicleState newState) override;
    int driveDirectionFactor; //factor to change requested motor speed direction based on what we currently consider the front

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    STATE_ESTIMATOR::StateEstimator* pStateEstimator;
    void (STATE_ESTIMATOR::StateEstimator::*setHeadingOffsetMethod)();
    VehicleState current_state;
    float waypointModeThreshold = 0; //if signal above this, we're move into waypoint mode
    float waypointIndexThreshold = 0.5; //if signal above this, reset the waypoint index
    float setHeadingThreshold = -0.5; //if signal below this, set the heading
    float setOriginThreshold = 0.5; //if signal above this, set the odometry origin

    NAVIGATION_MODE::Mode determineMode(float signal);
    bool shouldResetWaypointIndex(float signal);
    bool shouldSetHeading(float signal);
    bool shouldSetOdometryOrigin(float signal);
    WAYPOINTS::WaypointNavigation waypointNavigator;
    void setHeading(); //local method that's linked to the stateEstimator set_Heading_Offset method
    void setOrigin(); //local method that's linked to the stateEstimator set_Odometry_Offset method
    void parseTxSignals(ReceiverChannelValues signals); //function to use "spare" transmitter channels to as auxiliary inputs

};