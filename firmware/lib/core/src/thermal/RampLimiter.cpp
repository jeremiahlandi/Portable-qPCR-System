#include "core/thermal/RampLimiter.h"

#include <cmath>

namespace qpcr::thermal {

namespace {
/// A rate is a non-negative number of C/s; anything else holds the setpoint still.
double validRate(double rate) {
    return rate >= 0.0 ? rate : 0.0;
}
} // namespace

RampLimiter::RampLimiter(const RampLimiterConfig& config) : config_(config) {
    config_.maxHeatingRate = validRate(config_.maxHeatingRate);
    config_.maxCoolingRate = validRate(config_.maxCoolingRate);
}

void RampLimiter::reset(double time, double from) {
    value_ = from;
    time_ = time;
    started_ = true;
    settled_ = false;
}

void RampLimiter::hold(double time) {
    if (!started_) {
        return;
    }
    if (time > time_) {
        time_ = time;
    }
    settled_ = false;
}

double RampLimiter::update(double time, double target) {
    if (std::isnan(target)) {
        hold(time);
        return value_;
    }
    if (!started_) {
        reset(time, target);
        settled_ = true;
        return value_;
    }
    const double elapsed = time - time_;
    if (elapsed > 0.0) {
        const double delta = target - value_;
        const double maxUp = config_.maxHeatingRate * elapsed;
        const double maxDown = config_.maxCoolingRate * elapsed;
        if (delta > maxUp) {
            value_ += maxUp;
        } else if (delta < -maxDown) {
            value_ -= maxDown;
        } else {
            value_ = target; // within reach: land exactly, no floating-point residue
        }
        time_ = time;
    }
    settled_ = value_ == target;
    return value_;
}

} // namespace qpcr::thermal
