#include "core/thermal/PeltierController.h"

#include <algorithm>
#include <cmath>

namespace qpcr::thermal {

PeltierController::PeltierController(PeltierDrive& drive, const PeltierControllerConfig& config)
    : drive_(&drive), pid_(boundedBy(config.pid, drive.config())), ramp_(config.ramp) {}

PidConfig PeltierController::boundedBy(const PidConfig& pid, const PeltierDriveConfig& drive) {
    PidConfig bounded = pid;
    double ceiling = std::min(pid.maxPower, static_cast<double>(drive.maxDuty));
    if (!(ceiling > 0.0)) { // NaN or non-positive: the loop can only ever output zero
        ceiling = 0.0;
    }
    bounded.maxPower = std::min(ceiling, 1.0);
    bounded.minPower = -bounded.maxPower;
    return bounded;
}

bool PeltierController::start(double time, double temp) {
    if (!std::isfinite(temp)) {
        return false;
    }
    pid_.reset(time, temp);
    ramp_.reset(time, temp);
    output_ = 0.0;
    drive_->enable();
    running_ = true;
    return true;
}

void PeltierController::stop() {
    running_ = false;
    output_ = 0.0;
    drive_->disable();
}

double PeltierController::update(double readTime, double temp, double target) {
    if (!running_) {
        output_ = 0.0;
        return output_;
    }
    if (!std::isfinite(temp)) {
        ramp_.hold(readTime);
        output_ = 0.0;
        drive_->command(0.0F);
        return output_;
    }
    const double setpoint = ramp_.update(readTime, target);
    output_ = pid_.update(readTime, temp, setpoint);
    drive_->command(static_cast<float>(output_));
    return output_;
}

} // namespace qpcr::thermal
