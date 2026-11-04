#pragma once

// Setpoint ramp-rate limiter (issue #7). Sits between the protocol's target temperature and
// the PID so a step change in the setpoint reaches the controller as a ramp that never climbs
// faster than `maxHeatingRate` or falls faster than `maxCoolingRate`. The block then follows a
// slope the plant can track instead of being slammed at full power, which is what bounds the
// overshoot at a hold.

#include <limits>

namespace qpcr::thermal {

struct RampLimiterConfig {
    /// Fastest the ramped setpoint may rise, in C/s. Infinite (the default) means no limit;
    /// NaN or negative values are treated as 0, which holds the setpoint where it is.
    double maxHeatingRate = std::numeric_limits<double>::infinity();
    /// Fastest the ramped setpoint may fall, in C/s. Same rules as `maxHeatingRate`.
    double maxCoolingRate = std::numeric_limits<double>::infinity();
};

/// Moves a value toward a target at a bounded rate. Time is in seconds, values in Celsius.
class RampLimiter {
public:
    explicit RampLimiter(const RampLimiterConfig& config);

    /// Restarts the ramp from `from` at `time`; the next `update()` moves from here.
    void reset(double time, double from);

    /// Advances the ramped value toward `target` by at most rate x elapsed time and returns it.
    /// Before the first `reset()` the first call adopts the target outright. A NaN target holds
    /// (see `hold()`); a time that has not advanced past the last accepted one leaves the value
    /// where it is.
    double update(double time, double target);
    /// Keeps the value where it is and consumes the time up to `time`, so a later `update()`
    /// cannot catch up on the held interval in one step. Used while the reading is not usable.
    void hold(double time);

    /// The ramped setpoint as of the last `update()` or `reset()`.
    [[nodiscard]] double value() const { return value_; }
    /// True once the ramped value equals the last target handed to `update()`.
    [[nodiscard]] bool settled() const { return settled_; }
    [[nodiscard]] const RampLimiterConfig& config() const { return config_; }

private:
    RampLimiterConfig config_;
    bool started_ = false;
    bool settled_ = false;
    double value_ = 0.0;
    double time_ = 0.0;
};

} // namespace qpcr::thermal
