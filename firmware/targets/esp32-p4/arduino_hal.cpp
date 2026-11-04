#include "arduino_hal.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>

namespace qpcr::target {

bool LedcPwmOutput::attach() {
    if (!attached_) {
        attached_ = ledcAttach(static_cast<std::uint8_t>(pin_), frequencyHz_, resolutionBits_);
    }
    setDuty(0.0F);
    return attached_;
}

void LedcPwmOutput::setDuty(float value) {
    duty_ = std::clamp(value, 0.0F, 1.0F);
    if (!attached_) {
        return;
    }
    // Full scale is (2^bits - 1); the core maps that value to a fully on pin itself.
    const auto fullScale = static_cast<float>((1UL << resolutionBits_) - 1UL);
    ledcWrite(static_cast<std::uint8_t>(pin_),
              static_cast<std::uint32_t>(std::lround(duty_ * fullScale)));
}

void GpioOutput::begin(bool initial) {
    state_ = initial;
    begun_ = true;
    pinMode(static_cast<std::uint8_t>(pin_), OUTPUT);
    digitalWrite(static_cast<std::uint8_t>(pin_), initial ? HIGH : LOW);
}

void GpioOutput::set(bool on) {
    state_ = on;
    if (begun_) {
        digitalWrite(static_cast<std::uint8_t>(pin_), on ? HIGH : LOW);
    }
}

std::uint32_t ArduinoClock::millis() const {
    return static_cast<std::uint32_t>(::millis());
}

} // namespace qpcr::target
