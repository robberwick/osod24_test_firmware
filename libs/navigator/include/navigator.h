#include "receiver.h"
#include "statemanager.h"

class Navigator {
public:
    explicit Navigator(const Receiver* receiver, STATEMANAGER::StateManager *stateManager);
    ~Navigator();
    void navigate();

private:
    const Receiver *receiver{};
    STATEMANAGER::StateManager *pStateManager;
};