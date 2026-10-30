#pragma once

// Port of Klipper's heater verification (klippy/extras/verify_heater.py): trips when a
// heater that should be heating is not gaining temperature at the expected rate, or cannot
// hold its target. Ported from Klipper, Copyright (C) 2018 Kevin O'Connor, GNU GPLv3.

#include <cstdint>
#include <limits>

namespace qpcr::thermal {

struct HeaterCheckConfig {
    /// Band around the target, in °C, inside which the heater counts as "at target".
    double hysteresis = 5.0;
    /// Accumulated (target - hysteresis - temp) in °C·s that trips while holding.
    double maxError = 120.0;
    /// Temperature gain, in °C, expected within `checkGainTime` while approaching.
    double heatingGain = 2.0;
    /// Seconds allowed to achieve `heatingGain`.
    double checkGainTime = 20.0;
};

/// Call `check` about once per second with the current temperature and target.
class HeaterCheck {
public:
    enum class Status : std::uint8_t { Ok, Fault };

    explicit HeaterCheck(const HeaterCheckConfig& config) : config_(config) {}

    /// Evaluate one check. A Fault is sticky until `reset()`.
    Status check(double eventTime, double temp, double target);

    [[nodiscard]] bool faulted() const { return faulted_; }
    [[nodiscard]] double accumulatedError() const { return error_; }
    void reset();

private:
    static constexpr double kNever = std::numeric_limits<double>::infinity();

    HeaterCheckConfig config_;
    bool approachingTarget_ = false;
    bool startingApproach_ = false;
    double lastTarget_ = 0.0;
    double goalTemp_ = 0.0;
    double error_ = 0.0;
    double goalSystime_ = kNever;
    bool faulted_ = false;
};

} // namespace qpcr::thermal
