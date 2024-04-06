#pragma once
#include "SerialTransfer.h"
#include "payloads.h"

class Communicator {
public:
    static Communicator& getInstance();
    void setSerialTransfer(SerialTransfer* serialTransfer);

    #ifdef LOGGING_DISABLED
    template <typename T>
    void sendPacket(const T& packet) {}
    #else

    template <typename T>
    void sendPacket(const T& packet) {
        if (this->serialTransfer) {
            const uint16_t packetSize = this->serialTransfer->txObj(packet);
            this->serialTransfer->sendData(packetSize);
        }
    }
    #endif

    Communicator(const Communicator&) = delete;
    void operator=(const Communicator&) = delete;

private:
    explicit Communicator();

    ~Communicator();

    SerialTransfer *serialTransfer;
};