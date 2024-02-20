#include "types.h"
#include "drivetrain_config.h"

using namespace COMMON;
class WaypointNavigation {
public:
    ~WaypointNavigation();
    void navigate(State currentState);

private:
    float desiredV;  // desired velocity to get to next waypoint
    float desiredW;  // desired angular velocity to get to next waypoint
};