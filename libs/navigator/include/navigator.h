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
                        STATE_ESTIMATOR::StateEstimator* estimator);
    ~Navigator();
    void navigate();
    void update(const VehicleState newState) override;

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
    STATE_ESTIMATOR::StateEstimator* pStateEstimator;
    VehicleState current_state;
};