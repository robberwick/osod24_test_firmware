#include "receiver.h"
#include "statemanager.h"
#include "drivetrain_config.h"
#include "types.h"
#include "interfaces.h"
#include "drivetrain_config.h"
#include "waypoint_navigation.h"

using namespace COMMON;
class Navigator: public Observer {
public:
    explicit Navigator(const Receiver* receiver, STATEMANAGER::StateManager *stateManager, CONFIG::SteeringStyle direction);
    ~Navigator();
    void navigate();
    CONFIG::SteeringStyle driveDirection; //factor to change requested motor speed direction based on what we currently consider the front
    COMMON::NavigationMode navigationMode;
    void update(State newState) override;

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    State current_state;
    float waypointSignalThreshold = 0.5; //if signal above this, we're move into waypoint mode
    COMMON::NavigationMode determineMode(float signal);
    WAYPOINTS::WaypointNavigation waypointNavigator;
};