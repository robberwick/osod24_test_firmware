#include "receiver.h"
#include "statemanager.h"
#include "drivetrain_config.h"
#include "types.h"

class Navigator {
public:
    explicit Navigator(const Receiver* receiver, STATEMANAGER::StateManager *stateManager, CONFIG::SteeringStyle direction);
    ~Navigator();
    void navigate();
    CONFIG::SteeringStyle driveDirection; //factor to change requested motor speed direction based on what we currently consider the front
    COMMON::NavigationMode navigationMode;

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    float waypointSignalThreshold = 0.5; //if signal above this, we're move into waypoint mode
    COMMON::NavigationMode determineMode(float signal);
};