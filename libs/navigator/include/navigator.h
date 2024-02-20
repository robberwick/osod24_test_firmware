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
    COMMON::NavigationMode navigationMode;
    void update(State newState) override;

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    State current_state;
    float waypointSignalThreshold = 0.5; //if signal above this, we're move into waypoint mode
    COMMON::NavigationMode determineMode(float signal);
    WaypointNavigation waypointNavigator;
};