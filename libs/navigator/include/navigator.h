#include "receiver.h"
#include "statemanager.h"
#include "drivetrain_config.h"

class Navigator {
public:
    explicit Navigator(const Receiver* receiver, STATEMANAGER::StateManager *stateManager, CONFIG::SteeringStyle direction);
    ~Navigator();
    void navigate();
    CONFIG::SteeringStyle driveDirection; //factor to change requested motor speed direction based on what we currently consider the front

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
};