#ifndef ACCELERATED_ROTARY_ENCODER_H
#define ACCELERATED_ROTARY_ENCODER_H

#include <RotaryEncoder.h>

class AcceleratedRotaryEncoder : public RotaryEncoder
{
public:
    AcceleratedRotaryEncoder(
        int pin1,
        int pin2,
        LatchMode mode = LatchMode::FOUR0,
        unsigned long slowCutoffMs = 200,
        unsigned long fastCutoffMs = 5,
        long maximumMultiplier = 10);

    void setMaxPosition(long maxPosition)
    {
        maxPosition_ = maxPosition;
        hasMax_ = true;
    }
    void setMinPosition(long minPosition)
    {
        minPosition_ = minPosition;
        hasMin_ = true;
    }
    void tick();
    void tick(int sig1, int sig2);
    long getPosition();
    bool getNewPosition(long& newPosition);
    void setPosition(long newPosition);

private:
    void updateAcceleratedPosition();

    bool hasNewPosition_ = false;
    long maxPosition_;
    bool hasMax_;
    long minPosition_;
    bool hasMin_;
    unsigned long slowCutoffMs_;
    unsigned long fastCutoffMs_;
    long maximumMultiplier_;
    long rawPosition_;
    long acceleratedPosition_;
    int8_t lastDirection_;
};

#endif