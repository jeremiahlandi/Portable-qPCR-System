#pragma once

namespace qpcr::hal {

/// A two-state output (direction pin, LED, buzzer, relay).
class DigitalOutput {
public:
    virtual ~DigitalOutput() = default;
    virtual void set(bool on) = 0;
    [[nodiscard]] virtual bool isOn() const = 0;
};

} // namespace qpcr::hal
