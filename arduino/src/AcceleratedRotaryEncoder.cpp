#include "AcceleratedRotaryEncoder.h"

AcceleratedRotaryEncoder::AcceleratedRotaryEncoder(
    int pin1,
    int pin2,
    LatchMode mode,
    unsigned long slowCutoffMs,
    unsigned long fastCutoffMs,
    long maximumMultiplier)
    : RotaryEncoder(pin1, pin2, mode),
      slowCutoffMs_(slowCutoffMs),
      fastCutoffMs_(fastCutoffMs),
      maximumMultiplier_(maximumMultiplier),
      rawPosition_(RotaryEncoder::getPosition()),
      acceleratedPosition_(rawPosition_),
      lastDirection_(0)
{
}

void AcceleratedRotaryEncoder::tick()
{
    RotaryEncoder::tick();
    updateAcceleratedPosition();
}

void AcceleratedRotaryEncoder::tick(int sig1, int sig2)
{
    RotaryEncoder::tick(sig1, sig2);
    updateAcceleratedPosition();
}

long AcceleratedRotaryEncoder::getPosition()
{
    return acceleratedPosition_;
}

void AcceleratedRotaryEncoder::setPosition(long newPosition)
{
    RotaryEncoder::setPosition(newPosition);
    rawPosition_ = RotaryEncoder::getPosition();
    acceleratedPosition_ = newPosition;
    lastDirection_ = 0;
    hasNewPosition_ = true;
}

bool AcceleratedRotaryEncoder::getNewPosition(long& newPosition)
{
    if (hasNewPosition_) {
        newPosition = acceleratedPosition_;
        hasNewPosition_ = false;
        return true;
    }
    return false;
}

void AcceleratedRotaryEncoder::updateAcceleratedPosition()
{
    long newRawPosition = RotaryEncoder::getPosition();
    long rawDelta = newRawPosition - rawPosition_;
    if (rawDelta == 0) {
        return;
    }

    unsigned long intervalMs = getMillisBetweenRotations();
    long multiplier = 1;
    long lastAcceleratedPosition = acceleratedPosition_;
    int8_t direction = rawDelta > 0 ? 1 : -1;
    if (lastDirection_ == direction && intervalMs < slowCutoffMs_) {
        if (intervalMs < fastCutoffMs_) {
            intervalMs = fastCutoffMs_;
        }
        multiplier = 1 + ((slowCutoffMs_ - intervalMs) *
                          (maximumMultiplier_ - 1)) /
                         (slowCutoffMs_ - fastCutoffMs_);
        if (multiplier > maximumMultiplier_) {
            multiplier = maximumMultiplier_;
        }
    }

    acceleratedPosition_ += rawDelta * multiplier;
    if (hasMax_ && acceleratedPosition_ > maxPosition_) {
        acceleratedPosition_ = maxPosition_;
    }
    if (hasMin_ && acceleratedPosition_ < minPosition_) {
        acceleratedPosition_ = minPosition_;
    }
    if (acceleratedPosition_ != lastAcceleratedPosition) {
        hasNewPosition_ = true;
    }
    rawPosition_ = newRawPosition;
    lastDirection_ = direction;
}