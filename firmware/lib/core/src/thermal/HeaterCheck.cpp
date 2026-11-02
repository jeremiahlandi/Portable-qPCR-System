#include "core/thermal/HeaterCheck.h"

#include <algorithm>

namespace qpcr::thermal {

void HeaterCheck::reset() {
    approachingTarget_ = startingApproach_ = false;
    lastTarget_ = goalTemp_ = error_ = 0.0;
    goalSystime_ = kNever;
    faulted_ = false;
}

HeaterCheck::Status HeaterCheck::check(double eventTime, double temp, double target) {
    if (faulted_) {
        return Status::Fault;
    }
    if (temp >= target - config_.hysteresis || target <= 0.0) {
        // Temperature near target (or heater off): reset checks.
        approachingTarget_ = startingApproach_ = false;
        if (temp <= target + config_.hysteresis) {
            error_ = 0.0;
        }
        lastTarget_ = target;
        return Status::Ok;
    }
    error_ += (target - config_.hysteresis) - temp;
    if (!approachingTarget_) {
        if (target != lastTarget_) {
            // Target changed: start a fresh approach.
            approachingTarget_ = startingApproach_ = true;
            goalTemp_ = temp + config_.heatingGain;
            goalSystime_ = eventTime + config_.checkGainTime;
        } else if (error_ >= config_.maxError) {
            // Unable to maintain the target temperature.
            faulted_ = true;
            return Status::Fault;
        }
    } else if (temp >= goalTemp_) {
        // Still approaching: set the next goal.
        startingApproach_ = false;
        error_ = 0.0;
        goalTemp_ = temp + config_.heatingGain;
        goalSystime_ = eventTime + config_.checkGainTime;
    } else if (eventTime >= goalSystime_) {
        // No longer approaching the target.
        approachingTarget_ = false;
    } else if (startingApproach_) {
        goalTemp_ = std::min(goalTemp_, temp + config_.heatingGain);
    }
    lastTarget_ = target;
    return Status::Ok;
}

} // namespace qpcr::thermal
