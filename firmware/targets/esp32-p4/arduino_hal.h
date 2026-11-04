#pragma once

#include <cstdint>

#include "core/hal/Clock.h"
#include "core/hal/DigitalOutput.h"
#include "core/hal/PwmOutput.h"

// Arduino-ESP32 implementations of the core HAL interfaces for the ESP32-P4 target. Each
// output stays inert until its begin()/attach() call so that globals constructed before the
// Arduino runtime is up cannot touch the pins.

namespace qpcr::target {

/// One LEDC channel on a GPIO. setDuty() maps [0, 1] onto the full counter range.
class LedcPwmOutput : public hal::PwmOutput {
public:
    LedcPwmOutput(int pin, std::uint32_t frequencyHz, std::uint8_t resolutionBits)
        : pin_(pin), frequencyHz_(frequencyHz), resolutionBits_(resolutionBits) {}

    /// Claims an LEDC channel and drives the pin at zero duty. Returns false when no channel
    /// or timer is free; the output then ignores setDuty().
    bool attach();
    void setDuty(float value) override;
    [[nodiscard]] float duty() const override { return duty_; }
    [[nodiscard]] bool attached() const { return attached_; }

private:
    int pin_;
    std::uint32_t frequencyHz_;
    std::uint8_t resolutionBits_;
    bool attached_ = false;
    float duty_ = 0.0F;
};

/// A plain GPIO output.
class GpioOutput : public hal::DigitalOutput {
public:
    explicit GpioOutput(int pin) : pin_(pin) {}

    /// Configures the pin as an output at `initial`. Before this, set() only records the state.
    void begin(bool initial);
    void set(bool on) override;
    [[nodiscard]] bool isOn() const override { return state_; }

private:
    int pin_;
    bool begun_ = false;
    bool state_ = false;
};

/// Arduino millis() as a hal::Clock.
class ArduinoClock : public hal::Clock {
public:
    [[nodiscard]] std::uint32_t millis() const override;
};

} // namespace qpcr::target
