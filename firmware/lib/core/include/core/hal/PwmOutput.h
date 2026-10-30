#pragma once

namespace qpcr::hal {

/// A power output driven by a duty cycle in [0, 1]. Values outside are clamped.
class PwmOutput {
public:
    virtual ~PwmOutput() = default;
    virtual void setDuty(float duty) = 0;
    [[nodiscard]] virtual float duty() const = 0;
    /// Convenience: force the output off. Used by the safe state.
    void off() { setDuty(0.0F); }
};

}  // namespace qpcr::hal
