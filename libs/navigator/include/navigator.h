#include "receiver.h"
#include "statemanager.h"
#include "types.h"

class Navigator {
public:
    explicit Navigator(const Receiver* receiver, STATEMANAGER::StateManager *stateManager);
    ~Navigator();
    void navigate();
    COMMON::NavigationMode navigationMode;

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    float waypointSignalThreshold = 0.5; //if signal above this, we're move into waypoint mode
    COMMON::NavigationMode determineMode(float signal);
};