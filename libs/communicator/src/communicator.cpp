#include "communicator.h"
#include "payloads.h"

void Communicator::setSerialTransfer(SerialTransfer* serialTransfer) {
    this->serialTransfer = serialTransfer;
}

Communicator& Communicator::getInstance() {
    static Communicator instance;
    return instance;
}

Communicator::Communicator() = default;
Communicator::~Communicator() = default;
