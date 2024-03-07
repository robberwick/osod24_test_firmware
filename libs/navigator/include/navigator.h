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
    void update(const DriveTrainState newState) override;

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    STATE_ESTIMATOR::StateEstimator* pStateEstimator;
    DriveTrainState current_state;
};