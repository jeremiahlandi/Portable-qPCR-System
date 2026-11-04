#pragma once

// Block temperature control for the bidirectional Peltier (issue #7): the setpoint goes
// through a RampLimiter, the ported Klipper PID runs with a signed output bound, and the
// result is handed to PeltierDrive::command() as is. Positive output heats, negative cools.
//
// Nothing here decides *whether* the drive may run: the safety layer (issue #5) owns fault
// trips and calls PeltierDrive::disable() directly. This class only owns the loop between a
// valid reading and the bridge.

#include "core/thermal/PeltierDrive.h"
#include "core/thermal/Pid.h"
#include "core/thermal/RampLimiter.h"

namespace qpcr::thermal {

struct PeltierControllerConfig {
    /// Gains and smoothing for the loop. `maxPower` is the ceiling on |output|, further limited
    /// by the drive's duty cap so the integrator can never wind up above what the bridge will
    /// deliver. `minPower` is ignored: the controller always runs the PID symmetric.
    PidConfig pid;
    /// Setpoint ramp limits. Unlimited by default; see RampLimiter.
    RampLimiterConfig ramp;
};

/// Ramp -> signed PID -> H-bridge drive. Time is in seconds, temperatures in Celsius.
class PeltierController {
public:
    /// The drive must outlive the controller. Leaves the drive as it was (disabled at boot).
    PeltierController(PeltierDrive& drive, const PeltierControllerConfig& config);

    /// Arms the drive and starts the loop from the block's current temperature `temp` at
    /// `time`: the PID history is cleared and the setpoint ramp begins at `temp`. Returns false
    /// (and stays stopped) when `temp` is not a finite reading.
    bool start(double time, double temp);
    /// Disables the drive (safe state) and ignores updates until the next `start()`.
    void stop();
    [[nodiscard]] bool running() const { return running_; }

    /// One control step at sensor time `readTime`. Ramps the setpoint toward `target`, runs
    /// the PID and commands the drive. Returns the signed output in [-cap, +cap] where cap is
    /// the effective power ceiling; negative means cool. Returns 0 and leaves the drive alone
    /// while stopped. A non-finite `temp` releases the bridge for this step and holds the ramp
    /// (the safety layer, not the controller, decides whether that is a fault).
    double update(double readTime, double temp, double target);

    /// The last signed output handed to the drive.
    [[nodiscard]] double output() const { return output_; }
    /// The ramped setpoint the PID is tracking.
    [[nodiscard]] double rampedTarget() const { return ramp_.value(); }
    [[nodiscard]] const Pid& pid() const { return pid_; }
    [[nodiscard]] const RampLimiter& ramp() const { return ramp_; }

private:
    [[nodiscard]] static PidConfig boundedBy(const PidConfig& pid, const PeltierDriveConfig& drive);

    PeltierDrive* drive_ = nullptr;
    Pid pid_;
    RampLimiter ramp_;
    bool running_ = false;
    double output_ = 0.0;
};

} // namespace qpcr::thermal
