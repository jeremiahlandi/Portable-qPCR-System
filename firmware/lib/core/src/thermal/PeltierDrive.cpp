#include "core/thermal/PeltierDrive.h"

#include <algorithm>
#include <cmath>

namespace qpcr::thermal {

PeltierDrive::PeltierDrive(hal::PwmOutput& heat, hal::PwmOutput& cool, hal::DigitalOutput& enable,
                           const hal::Clock& clock, const PeltierDriveConfig& config)
    : heat_(&heat), cool_(&cool), enable_(&enable), clock_(&clock), config_(config) {
    config_.maxDuty = std::clamp(config_.maxDuty, 0.0F, 1.0F);
    release();
}

void PeltierDrive::enable() {
    commanded_ = 0.0F;
    waiting_ = false;
    enabled_ = true;
}

void PeltierDrive::disable() {
    enabled_ = false;
    waiting_ = false;
    commanded_ = 0.0F;
    release();
}

void PeltierDrive::command(float value) {
    if (std::isnan(value)) {
        value = 0.0F;
    }
    commanded_ = std::clamp(value, -1.0F, 1.0F);
    tick();
}

void PeltierDrive::tick() {
    if (!enabled_) {
        release();
        return;
    }
    const Side want = sideOf(commanded_);
    const float duty = std::min(std::fabs(commanded_), config_.maxDuty);
    if (want == Side::Off || duty <= 0.0F) {
        waiting_ = false;
        release();
        return;
    }
    // Changing over: let go of the side that is driving before anything else happens.
    if (side_ != Side::Off && side_ != want) {
        release();
    }
    if (lastDriven_ != Side::Off && lastDriven_ != want &&
        clock_->elapsedSince(releasedAtMs_) < config_.deadTimeMs) {
        waiting_ = true;
        return;
    }
    waiting_ = false;
    drive(want, duty);
}

PeltierDrive::Side PeltierDrive::sideOf(float value) {
    if (value > 0.0F) {
        return Side::Heat;
    }
    if (value < 0.0F) {
        return Side::Cool;
    }
    return Side::Off;
}

void PeltierDrive::release() {
    if (side_ != Side::Off) {
        releasedAtMs_ = clock_->millis();
    }
    side_ = Side::Off;
    applied_ = 0.0F;
    heat_->off();
    cool_->off();
    enable_->set(false);
}

void PeltierDrive::drive(Side newSide, float duty) {
    // Set the duties first so the enable line only ever turns on a known pair of inputs.
    if (newSide == Side::Heat) {
        cool_->off();
        heat_->setDuty(duty);
        applied_ = duty;
    } else {
        heat_->off();
        cool_->setDuty(duty);
        applied_ = -duty;
    }
    enable_->set(true);
    side_ = newSide;
    lastDriven_ = newSide;
}

} // namespace qpcr::thermal
