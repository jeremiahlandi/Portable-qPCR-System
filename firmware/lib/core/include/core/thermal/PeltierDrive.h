#pragma once

// Bidirectional Peltier drive through a dual half-bridge such as the IBT_2 / BTS7960 (issue #7).
// One PWM output per direction (RPWM heats, LPWM cools) plus a shared enable line. The drive
// takes a signed command, keeps either side under a duty cap and never lets the two sides
// change over while anything is still driving: a reversal first releases the bridge (both PWMs
// at zero, enable low) and only drives the other side once the dead time has elapsed.
//
// Board-agnostic: the PID loop (#4), the safety layer (#5) and the target glue sit on top.

#include <cstdint>

#include "core/hal/Clock.h"
#include "core/hal/DigitalOutput.h"
#include "core/hal/PwmOutput.h"

namespace qpcr::thermal {

/// Highest duty the default configuration lets either side run at. Sized for the CUI
/// CP303385H (3.0 A max, about 4 ohm) on a 15 V supply: 15 V x 0.75 / 4 ohm = 2.8 A.
constexpr float kPeltierDefaultMaxDuty = 0.75F;

/// Default pause between releasing one side and driving the other, in milliseconds. Long enough
/// for the output filter to discharge; the exact value is confirmed on a scope (#7).
constexpr std::uint32_t kPeltierDefaultDeadTimeMs = 50;

struct PeltierDriveConfig {
    /// Hard ceiling on the duty of either side, in [0, 1]. Commands above it are clamped, not
    /// scaled, so a command below the cap is the duty that reaches the pin. Values outside
    /// [0, 1] are clamped when the drive is constructed; 0 disables the output entirely.
    float maxDuty = kPeltierDefaultMaxDuty;
    /// Milliseconds the bridge stays released between driving opposite sides. 0 reverses
    /// immediately (both PWMs are still written to zero before the other side starts).
    std::uint32_t deadTimeMs = kPeltierDefaultDeadTimeMs;
};

/// Maps a signed command in [-1, 1] onto heat / cool PWM outputs and an enable line.
///
/// Lifecycle: constructed disabled with the pins in the safe state. `enable()` arms the drive,
/// `command()` sets the signed duty and applies it, `tick()` re-applies a pending command once
/// the dead time has passed (call it from the control loop between commands), `disable()`
/// returns to the safe state and ignores commands until the next `enable()`.
class PeltierDrive {
public:
    /// Which side is currently driving. `Off` while released, in dead time or disabled.
    enum class Side : std::uint8_t { Off, Heat, Cool };

    /// Writes the safe state (both PWMs zero, enable low) to the outputs. The outputs and the
    /// clock must outlive the drive.
    PeltierDrive(hal::PwmOutput& heat, hal::PwmOutput& cool, hal::DigitalOutput& enable,
                 const hal::Clock& clock, const PeltierDriveConfig& config = {});

    /// Arms the drive. Clears any command issued while disabled; nothing drives until the next
    /// `command()`.
    void enable();
    /// Safe state: both PWMs zero, enable low, pending command cleared. Sticky until `enable()`.
    void disable();
    [[nodiscard]] bool isEnabled() const { return enabled_; }

    /// Signed duty request: positive heats, negative cools, zero releases the bridge. Clamped
    /// to [-1, 1]; NaN counts as zero. Applied immediately unless a reversal is in its dead
    /// time, in which case it is held and applied by a later `command()` or `tick()`.
    void command(float value);
    /// Re-evaluates the pending command against the clock. Idempotent.
    void tick();

    /// The clamped request, before the cap.
    [[nodiscard]] float commanded() const { return commanded_; }
    /// The signed duty on the pins right now: positive on the heat PWM, negative on the cool
    /// PWM, zero while released.
    [[nodiscard]] float applied() const { return applied_; }
    [[nodiscard]] Side side() const { return side_; }
    /// True while a command is held back because the other side drove too recently.
    [[nodiscard]] bool inDeadTime() const { return waiting_; }
    [[nodiscard]] const PeltierDriveConfig& config() const { return config_; }

private:
    [[nodiscard]] static Side sideOf(float value);
    /// Both PWMs to zero, enable low. Starts the dead time if a side was driving.
    void release();
    void drive(Side newSide, float duty);

    hal::PwmOutput* heat_ = nullptr;
    hal::PwmOutput* cool_ = nullptr;
    hal::DigitalOutput* enable_ = nullptr;
    const hal::Clock* clock_ = nullptr;
    PeltierDriveConfig config_;

    bool enabled_ = false;
    bool waiting_ = false;
    float commanded_ = 0.0F;
    float applied_ = 0.0F;
    Side side_ = Side::Off;
    /// Last side that drove, kept across releases so a reversal through zero still waits.
    Side lastDriven_ = Side::Off;
    std::uint32_t releasedAtMs_ = 0;
};

} // namespace qpcr::thermal
