#pragma once

// Port of the temperature smoothing in Klipper's Heater.temperature_callback
// (klippy/extras/heaters.py). Ported from Klipper, Copyright (C) 2016-2025 Kevin O'Connor,
// GNU GPLv3.

#include <algorithm>

namespace qpcr::thermal {

/// First-order smoothing of a sampled temperature, used for "is it settled" decisions.
class TemperatureSmoother {
public:
    explicit TemperatureSmoother(double smoothTime) : invSmoothTime_(1.0 / smoothTime) {}

    /// Feed one sample; returns the smoothed value.
    double update(double readTime, double temp) {
        const double timeDiff = readTime - lastTempTime_;
        lastTempTime_ = readTime;
        const double tempDiff = temp - smoothed_;
        const double adjTime = std::min(timeDiff * invSmoothTime_, 1.0);
        smoothed_ += tempDiff * adjTime;
        return smoothed_;
    }

    [[nodiscard]] double value() const { return smoothed_; }

private:
    double invSmoothTime_;
    double smoothed_ = 0.0;
    double lastTempTime_ = 0.0;
};

} // namespace qpcr::thermal
