#pragma once

// Port of Klipper's ControlPID (klippy/extras/heaters.py).
// Ported from Klipper, Copyright (C) 2016-2025 Kevin O'Connor, GNU GPLv3.

namespace qpcr::thermal {

struct PidConfig {
    /// Gains in Klipper's config units (scaled by 255, see KlipperPort.h).
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    /// Upper bound of the control output, in [0, 1].
    double maxPower = 1.0;
    /// Minimum time base for the derivative term, seconds (Klipper `smooth_time`).
    double smoothTime = 1.0;
};

/// Heat-only PID with a bounded integral and a smoothed derivative.
/// Time is in seconds, temperatures in Celsius. Output is in [0, maxPower].
class Pid {
public:
    explicit Pid(const PidConfig& config);

    /// One controller step. `readTime` is the sensor's sample time.
    /// Returns the bounded control output.
    double update(double readTime, double temp, double target);

    /// True while the loop is still settling (Klipper `check_busy`).
    [[nodiscard]] bool busy(double smoothedTemp, double target) const;

    [[nodiscard]] double lastDerivative() const { return prevTempDeriv_; }
    [[nodiscard]] double lastIntegral() const { return prevTempInteg_; }
    [[nodiscard]] const PidConfig& config() const { return config_; }

    /// Forget all history (fresh start, as if just constructed).
    void reset();

private:
    PidConfig config_;
    double kp_;
    double ki_;
    double kd_;
    double tempIntegMax_;
    double prevTemp_;
    double prevTempTime_ = 0.0;
    double prevTempDeriv_ = 0.0;
    double prevTempInteg_ = 0.0;
};

} // namespace qpcr::thermal
