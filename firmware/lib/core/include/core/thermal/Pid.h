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
    /// Lower bound of the control output, in [-1, 0]. Klipper's heater keeps 0 (heat only);
    /// a bidirectional drive such as the Peltier H-bridge (issue #7) sets -maxPower so that a
    /// negative output means cool. The integral is bounded to [minPower, maxPower] / ki the
    /// same way. NaN or positive values fall back to 0.
    double minPower = 0.0;
};

/// PID with a bounded integral and a smoothed derivative. Heat-only as ported from Klipper;
/// a negative `minPower` extends the same loop to a signed output (issue #7).
/// Time is in seconds, temperatures in Celsius. Output is in [minPower, maxPower].
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

    /// Forget all history (fresh start, as if just constructed: ambient at time zero).
    void reset();
    /// Forget all history and continue from `temp` at `readTime`, so a loop that starts late
    /// in the uptime does not see one huge first time step that winds the integral up.
    void reset(double readTime, double temp);

private:
    PidConfig config_;
    double kp_;
    double ki_;
    double kd_;
    double tempIntegMax_;
    double tempIntegMin_;
    double prevTemp_;
    double prevTempTime_ = 0.0;
    double prevTempDeriv_ = 0.0;
    double prevTempInteg_ = 0.0;
};

} // namespace qpcr::thermal
