#include "core/thermal/Pid.h"

#include <algorithm>
#include <cmath>

#include "core/thermal/KlipperPort.h"

namespace qpcr::thermal {

namespace {
constexpr double kPidSettleDelta = 1.0;
constexpr double kPidSettleSlope = 0.1;
} // namespace

Pid::Pid(const PidConfig& config)
    : config_(config), kp_(config.kp / klipper::kPidParamBase),
      ki_(config.ki / klipper::kPidParamBase), kd_(config.kd / klipper::kPidParamBase),
      tempIntegMax_(ki_ != 0.0 ? config.maxPower / ki_ : 0.0), prevTemp_(klipper::kAmbientTemp) {}

void Pid::reset() {
    prevTemp_ = klipper::kAmbientTemp;
    prevTempTime_ = 0.0;
    prevTempDeriv_ = 0.0;
    prevTempInteg_ = 0.0;
}

double Pid::update(double readTime, double temp, double target) {
    const double timeDiff = readTime - prevTempTime_;
    // Change of temperature, with the derivative smoothed over `smoothTime`.
    const double tempDiff = temp - prevTemp_;
    double tempDeriv = 0.0;
    if (timeDiff >= config_.smoothTime) {
        tempDeriv = tempDiff / timeDiff;
    } else {
        tempDeriv =
            (prevTempDeriv_ * (config_.smoothTime - timeDiff) + tempDiff) / config_.smoothTime;
    }
    // Accumulated error, bounded so the integral alone cannot exceed max power.
    const double tempErr = target - temp;
    double tempInteg = prevTempInteg_ + tempErr * timeDiff;
    tempInteg = std::max(0.0, std::min(tempIntegMax_, tempInteg));
    // Output.
    const double co = kp_ * tempErr + ki_ * tempInteg - kd_ * tempDeriv;
    const double boundedCo = std::max(0.0, std::min(config_.maxPower, co));
    // State for the next sample. The integral is only kept when the output was not
    // saturated (anti-windup).
    prevTemp_ = temp;
    prevTempTime_ = readTime;
    prevTempDeriv_ = tempDeriv;
    if (co == boundedCo) {
        prevTempInteg_ = tempInteg;
    }
    return boundedCo;
}

bool Pid::busy(double smoothedTemp, double target) const {
    const double tempDiff = target - smoothedTemp;
    return std::fabs(tempDiff) > kPidSettleDelta || std::fabs(prevTempDeriv_) > kPidSettleSlope;
}

} // namespace qpcr::thermal
