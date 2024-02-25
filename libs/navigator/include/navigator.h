#include "receiver.h"
#include "statemanager.h"
#include "types.h"
#include "interfaces.h"
#include "drivetrain_config.h"
#include "waypoint_navigation.h"

using namespace COMMON;
class Navigator: public Observer {
public:
    explicit Navigator(const Receiver* receiver, STATEMANAGER::StateManager *stateManager);
    ~Navigator();
    void navigate();
    NAVIGATION_MODE::Mode navigationMode;
    void update(VehicleState newState) override;

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    VehicleState current_state;
    float waypointModeThreshold = 0; //if signal above this, we're move into waypoint mode
    float waypointIndexThreshold = 0.5; //if signal above this, reset the waypoint index
    
    NAVIGATION_MODE::Mode determineMode(float signal);
    bool shouldResetWaypointIndex(float signal);
    WAYPOINTS::WaypointNavigation waypointNavigator;
};