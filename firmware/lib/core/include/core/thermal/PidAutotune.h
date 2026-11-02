#pragma once

// Port of Klipper's PID autotune (ControlAutoTune in klippy/extras/pid_calibrate.py):
// relay oscillation around the target, Åström-Hägglund estimate of Ku/Tu, Ziegler-Nichols
// gains. Ported from Klipper, Copyright (C) 2016-2018 Kevin O'Connor, GNU GPLv3.

#include <array>
#include <cstddef>

namespace qpcr::thermal {

struct PidGains {
    /// In Klipper config units (scaled by 255), the same units PidConfig takes.
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
};

/// Drive the heater with this controller until `busy()` is false, then read `finalGains()`.
class PidAutotune {
public:
    static constexpr std::size_t kMaxPeaks = 64;

    PidAutotune(double calibrateTemp, double maxPower);

    /// One step: returns the power to apply (0 or maxPower).
    double update(double readTime, double temp);

    /// True until enough oscillation peaks have been recorded.
    [[nodiscard]] bool busy() const;
    /// The target the relay is currently aiming at (alternates below the calibrate temp).
    [[nodiscard]] double activeTarget() const { return activeTarget_; }
    [[nodiscard]] std::size_t peakCount() const { return peakCount_; }

    /// Gains from the median oscillation period. Only meaningful when `!busy()`.
    [[nodiscard]] PidGains finalGains() const;

private:
    struct Peak {
        double time;
        double temp;
    };

    void checkPeaks();
    [[nodiscard]] PidGains calcPid(std::size_t pos) const;

    double maxPower_;
    double calibrateTemp_;
    double activeTarget_;
    bool heating_ = false;
    double peak_ = 0.0;
    double peakTime_ = 0.0;
    std::array<Peak, kMaxPeaks> peaks_{};
    std::size_t peakCount_ = 0;
};

} // namespace qpcr::thermal
