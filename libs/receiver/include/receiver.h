#pragma once

struct ReceiverChannelValues {
    float AIL;
    float ELE;
    float THR;
    float RUD;
    float AUX;
    float NC;
};

enum class RX_CHANNELS {
    AIL = 0,
    ELE = 1,
    THR = 2,
    RUD = 3,
    AUX = 4,
    NC = 5
};
extern const char *RX_CHANNEL_NAMES[];

class Receiver {
public:
    // constructor
    explicit Receiver(int pin);

    Receiver();

    // destructor
    virtual ~Receiver();

    [[nodiscard]] virtual ReceiverChannelValues get_channel_values() const;

    [[nodiscard]] virtual bool get_receiver_data() const;
};

Receiver* getReceiver(int pin);


float map_value_to_range(int value);

float map_range(float a1, float a2, float b1, float b2, float s);
