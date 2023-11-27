#include "receiver.h"

class Navigator {
public:
    explicit Navigator(const Receiver* receiver);
    ~Navigator();
    void navigate();

private:
    const Receiver *receiver{};
};