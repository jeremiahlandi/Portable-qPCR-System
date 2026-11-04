// Host tests for the signed thermal control chain (issue #7): the Klipper PID with a negative
// output bound, the setpoint ramp limiter and the PeltierController that feeds the H-bridge
// drive, all through the fakes.
#include <unity.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "core/thermal/PeltierController.h"
#include "core/thermal/PeltierDrive.h"
#include "core/thermal/Pid.h"
#include "core/thermal/RampLimiter.h"
#include "fakes/FakeClock.h"
#include "fakes/FakeDigitalOutput.h"
#include "fakes/FakePwmOutput.h"

using namespace qpcr;
using thermal::PeltierController;
using thermal::PeltierControllerConfig;
using thermal::PeltierDrive;
using thermal::PeltierDriveConfig;
using thermal::Pid;
using thermal::PidConfig;
using thermal::RampLimiter;
using thermal::RampLimiterConfig;

namespace {

constexpr double kTol = 1e-9;
constexpr float kDutyTol = 0.0005F;
constexpr double kDt = 0.25;
constexpr std::uint32_t kDeadTimeMs = 50;
constexpr double kInf = std::numeric_limits<double>::infinity();

/// Klipper scales gains by 255, so kp = 255 is a unit proportional gain: output = error in C.
PidConfig unitP() {
    PidConfig config;
    config.kp = 255.0;
    config.maxPower = 1.0;
    return config;
}

PidConfig integralOnly(double ki) {
    PidConfig config;
    config.ki = ki;
    config.maxPower = 1.0;
    return config;
}

struct Rig {
    fakes::FakePwmOutput heat;
    fakes::FakePwmOutput cool;
    fakes::FakeDigitalOutput enable;
    fakes::FakeClock clock;
    PeltierDrive drive;
    PeltierController controller;

    explicit Rig(const PeltierControllerConfig& config, const PeltierDriveConfig& driveConfig = {})
        : drive(heat, cool, enable, clock, driveConfig), controller(drive, config) {}

    [[nodiscard]] bool isSafe() const {
        return heat.duty() == 0.0F && cool.duty() == 0.0F && !enable.isOn();
    }
};

PeltierControllerConfig unitPController() {
    PeltierControllerConfig config;
    config.pid = unitP();
    return config;
}

} // namespace

void setUp() {}
void tearDown() {}

// --- Pid: signed output bound ------------------------------------------------------------------

void test_pid_default_lower_bound_is_zero_heat_only() {
    Pid pid(unitP());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, pid.config().minPower);
    pid.reset(0.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, pid.update(kDt, 60.0, 55.0));
}

void test_pid_negative_bound_gives_negative_output_above_target() {
    PidConfig config = unitP();
    config.minPower = -1.0;
    Pid pid(config);
    pid.reset(0.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.3, pid.update(kDt, 60.0, 59.7));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.3, pid.update(2 * kDt, 60.0, 60.3));
}

void test_pid_output_saturates_at_min_and_max_power() {
    PidConfig config = unitP();
    config.maxPower = 0.6;
    config.minPower = -0.4;
    Pid pid(config);
    pid.reset(0.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.6, pid.update(kDt, 60.0, 95.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.4, pid.update(2 * kDt, 60.0, 25.0));
}

void test_pid_integral_accumulates_negative_down_to_min_power() {
    PidConfig config = integralOnly(255.0); // ki = 1 after Klipper scaling
    config.minPower = -0.5;
    Pid pid(config);
    pid.reset(0.0, 61.0);
    // One degree above target: the integral falls by 1 C x dt per sample.
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -kDt, pid.update(kDt, 61.0, 60.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -kDt, pid.lastIntegral());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -2 * kDt, pid.update(2 * kDt, 61.0, 60.0));
    // Bounded at minPower / ki so the integral alone can never ask for more than the drive gives.
    for (int i = 3; i < 20; ++i) {
        pid.update(i * kDt, 61.0, 60.0);
    }
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.5, pid.lastIntegral());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.5, pid.update(20 * kDt, 61.0, 60.0));
}

void test_pid_integral_unwinds_through_zero_on_reversal() {
    PidConfig config = integralOnly(255.0);
    config.minPower = -1.0;
    Pid pid(config);
    pid.reset(0.0, 60.0);
    pid.update(kDt, 59.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, kDt, pid.lastIntegral());
    pid.update(2 * kDt, 61.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, pid.lastIntegral());
    pid.update(3 * kDt, 61.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -kDt, pid.lastIntegral());
}

void test_pid_invalid_min_power_falls_back_to_heat_only() {
    PidConfig positive = unitP();
    positive.minPower = 0.5;
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, Pid(positive).config().minPower);
    PidConfig nan = unitP();
    nan.minPower = std::nan("");
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, Pid(nan).config().minPower);
    PidConfig tooLow = unitP();
    tooLow.minPower = -3.0;
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -1.0, Pid(tooLow).config().minPower);
}

void test_pid_reset_at_a_time_and_temperature_does_not_wind_the_integral() {
    PidConfig config = integralOnly(255.0);
    config.minPower = -1.0;
    Pid pid(config);
    // Klipper's reset assumes time zero at ambient; a run that starts late must not see a
    // huge first time step that slams the integral into its bound.
    pid.reset(1000.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, pid.lastIntegral());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, pid.lastDerivative());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, kDt, pid.update(1000.0 + kDt, 60.0, 61.0));
}

// --- RampLimiter -------------------------------------------------------------------------------

void test_ramp_first_update_without_reset_adopts_the_target() {
    RampLimiterConfig config;
    config.maxHeatingRate = 1.0;
    config.maxCoolingRate = 1.0;
    RampLimiter ramp(config);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 95.0, ramp.update(10.0, 95.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 95.0, ramp.value());
    TEST_ASSERT_TRUE(ramp.settled());
}

void test_ramp_rises_no_faster_than_the_heating_rate_and_lands_exactly() {
    RampLimiterConfig config;
    config.maxHeatingRate = 2.0;
    config.maxCoolingRate = 1.0;
    RampLimiter ramp(config);
    ramp.reset(0.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.5, ramp.update(kDt, 61.2));
    TEST_ASSERT_FALSE(ramp.settled());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 61.0, ramp.update(2 * kDt, 61.2));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 61.2, ramp.update(3 * kDt, 61.2)); // last step is shorter
    TEST_ASSERT_TRUE(ramp.settled());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 61.2, ramp.update(4 * kDt, 61.2));
}

void test_ramp_falls_no_faster_than_the_cooling_rate() {
    RampLimiterConfig config;
    config.maxHeatingRate = 2.0;
    config.maxCoolingRate = 1.0;
    RampLimiter ramp(config);
    ramp.reset(0.0, 95.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 94.75, ramp.update(kDt, 60.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 94.5, ramp.update(2 * kDt, 60.0));
}

void test_ramp_step_scales_with_the_sample_interval() {
    RampLimiterConfig config;
    config.maxHeatingRate = 2.0;
    config.maxCoolingRate = 2.0;
    RampLimiter ramp(config);
    ramp.reset(0.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 62.0, ramp.update(1.0, 95.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 62.2, ramp.update(1.1, 95.0));
}

void test_ramp_default_is_unlimited() {
    const RampLimiterConfig config;
    TEST_ASSERT_TRUE(std::isinf(config.maxHeatingRate));
    TEST_ASSERT_TRUE(std::isinf(config.maxCoolingRate));
    RampLimiter ramp(config);
    ramp.reset(0.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 95.0, ramp.update(kDt, 95.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 25.0, ramp.update(2 * kDt, 25.0));
}

void test_ramp_holds_when_time_does_not_advance() {
    RampLimiterConfig config;
    config.maxHeatingRate = 2.0;
    config.maxCoolingRate = 2.0;
    RampLimiter ramp(config);
    ramp.reset(1.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.0, ramp.update(1.0, 95.0)); // same instant
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.0, ramp.update(0.5, 95.0)); // clock went backwards
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.5, ramp.update(1.25, 95.0));
}

void test_ramp_holds_on_nan_target() {
    RampLimiterConfig config;
    config.maxHeatingRate = 2.0;
    config.maxCoolingRate = 2.0;
    RampLimiter ramp(config);
    ramp.reset(0.0, 60.0);
    ramp.update(kDt, 95.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.5, ramp.update(2 * kDt, std::nan("")));
    TEST_ASSERT_FALSE(ramp.settled());
    // The held interval is spent, not banked: the next step is one sample's worth.
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 61.0, ramp.update(3 * kDt, 95.0));
}

void test_ramp_hold_consumes_time_without_moving() {
    RampLimiterConfig config;
    config.maxHeatingRate = 2.0;
    config.maxCoolingRate = 2.0;
    RampLimiter ramp(config);
    ramp.reset(0.0, 60.0);
    ramp.hold(2.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.0, ramp.value());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.5, ramp.update(2.0 + kDt, 95.0));
    ramp.hold(1.0); // does not run time backwards
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 61.0, ramp.update(2.0 + 2 * kDt, 95.0));
}

void test_ramp_follows_a_target_that_changes_mid_ramp() {
    RampLimiterConfig config;
    config.maxHeatingRate = 2.0;
    config.maxCoolingRate = 2.0;
    RampLimiter ramp(config);
    ramp.reset(0.0, 60.0);
    ramp.update(kDt, 95.0);
    ramp.update(2 * kDt, 95.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 61.0, ramp.value());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.5, ramp.update(3 * kDt, 25.0));
}

void test_ramp_reset_restarts_from_the_given_value() {
    RampLimiterConfig config;
    config.maxHeatingRate = 2.0;
    config.maxCoolingRate = 2.0;
    RampLimiter ramp(config);
    ramp.reset(0.0, 60.0);
    ramp.update(kDt, 95.0);
    ramp.reset(10.0, 80.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 80.0, ramp.value());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 80.5, ramp.update(10.0 + kDt, 95.0));
}

void test_ramp_invalid_rates_hold_the_setpoint() {
    RampLimiterConfig negative;
    negative.maxHeatingRate = -1.0;
    negative.maxCoolingRate = std::nan("");
    RampLimiter ramp(negative);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, ramp.config().maxHeatingRate);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, ramp.config().maxCoolingRate);
    ramp.reset(0.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.0, ramp.update(kDt, 95.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.0, ramp.update(2 * kDt, 25.0));
}

// --- PeltierController: lifecycle --------------------------------------------------------------

void test_controller_commands_nothing_before_start() {
    Rig rig(unitPController());
    TEST_ASSERT_FALSE(rig.controller.running());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, rig.controller.update(kDt, 60.0, 95.0));
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_FALSE(rig.drive.isEnabled());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, rig.controller.output());
}

void test_start_arms_the_drive_without_driving() {
    Rig rig(unitPController());
    rig.controller.start(0.0, 60.0);
    TEST_ASSERT_TRUE(rig.controller.running());
    TEST_ASSERT_TRUE(rig.drive.isEnabled());
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.0, rig.controller.rampedTarget());
}

void test_stop_puts_the_drive_in_its_safe_state_and_ignores_updates() {
    Rig rig(unitPController());
    rig.controller.start(0.0, 60.0);
    rig.controller.update(kDt, 60.0, 60.5);
    TEST_ASSERT_FALSE(rig.isSafe());
    rig.controller.stop();
    TEST_ASSERT_FALSE(rig.controller.running());
    TEST_ASSERT_FALSE(rig.drive.isEnabled());
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, rig.controller.output());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, rig.controller.update(2 * kDt, 60.0, 60.5));
    TEST_ASSERT_TRUE(rig.isSafe());
}

void test_start_late_in_uptime_does_not_wind_the_integral() {
    PeltierControllerConfig config;
    config.pid = integralOnly(255.0);
    Rig rig(config);
    rig.controller.start(3600.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, kDt, rig.controller.update(3600.0 + kDt, 60.0, 61.0));
}

// --- PeltierController: sign mapping and saturation --------------------------------------------

void test_below_target_heats_with_the_pid_output_as_duty() {
    Rig rig(unitPController());
    rig.controller.start(0.0, 60.0);
    const double out = rig.controller.update(kDt, 60.0, 60.4);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.4, out);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.4, rig.controller.output());
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.4F, rig.heat.duty());
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.0F, rig.cool.duty());
    TEST_ASSERT_TRUE(rig.enable.isOn());
    TEST_ASSERT_EQUAL(static_cast<int>(PeltierDrive::Side::Heat),
                      static_cast<int>(rig.drive.side()));
}

void test_above_target_cools_with_the_negated_pid_output_as_duty() {
    Rig rig(unitPController());
    rig.controller.start(0.0, 60.0);
    const double out = rig.controller.update(kDt, 60.0, 59.7);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.3, out);
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.0F, rig.heat.duty());
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.3F, rig.cool.duty());
    TEST_ASSERT_TRUE(rig.enable.isOn());
    TEST_ASSERT_EQUAL(static_cast<int>(PeltierDrive::Side::Cool),
                      static_cast<int>(rig.drive.side()));
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, -0.3F, rig.drive.applied());
}

void test_at_target_releases_the_bridge() {
    Rig rig(unitPController());
    rig.controller.start(0.0, 60.0);
    rig.controller.update(kDt, 60.0, 60.4);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, rig.controller.update(2 * kDt, 60.0, 60.0));
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_TRUE(rig.drive.isEnabled());
}

void test_output_saturates_at_the_drive_duty_cap_in_both_directions() {
    // pid.maxPower is 1.0 but the drive clamps at kPeltierDefaultMaxDuty (0.75): the controller
    // bounds the PID there so the integrator cannot wind up above what the bridge delivers.
    Rig rig(unitPController());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.75, rig.controller.pid().config().maxPower);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.75, rig.controller.pid().config().minPower);
    rig.controller.start(0.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.75, rig.controller.update(kDt, 60.0, 95.0));
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.75F, rig.heat.duty());
    rig.controller.update(2 * kDt, 60.0, 60.0);
    rig.clock.advance(1000);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.75, rig.controller.update(3 * kDt, 95.0, 60.0));
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.75F, rig.cool.duty());
}

void test_a_lower_pid_max_power_wins_over_the_drive_cap() {
    PeltierControllerConfig config = unitPController();
    config.pid.maxPower = 0.5;
    Rig rig(config);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.5, rig.controller.pid().config().maxPower);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.5, rig.controller.pid().config().minPower);
    rig.controller.start(0.0, 60.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.5, rig.controller.update(kDt, 60.0, 95.0));
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.5F, rig.heat.duty());
}

void test_output_sign_always_matches_the_side_driving() {
    Rig rig(unitPController());
    rig.controller.start(0.0, 60.0);
    const double targets[] = {60.2, 60.0, 59.8, 59.8, 60.0, 60.5};
    double time = 0.0;
    for (const double target : targets) {
        time += kDt;
        rig.clock.advance(1000);
        const double out = rig.controller.update(time, 60.0, target);
        const float applied = rig.drive.applied();
        if (out > 0.0) {
            TEST_ASSERT_TRUE(applied > 0.0F);
        } else if (out < 0.0) {
            TEST_ASSERT_TRUE(applied < 0.0F);
        } else {
            TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.0F, applied);
        }
    }
}

// --- PeltierController: ramp limiting ----------------------------------------------------------

void test_setpoint_step_is_ramped_and_the_pid_tracks_the_ramp() {
    PeltierControllerConfig config = unitPController();
    config.ramp.maxHeatingRate = 2.0;
    config.ramp.maxCoolingRate = 1.0;
    Rig rig(config);
    rig.controller.start(0.0, 60.0);
    // A 35 C step: the ramped target moves 0.5 C per 0.25 s sample, so the error the PID sees
    // (block held at 60 C) grows by 0.5 each sample instead of jumping to 35.
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.5, rig.controller.update(kDt, 60.0, 95.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.5, rig.controller.rampedTarget());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.75, rig.controller.update(2 * kDt, 60.0, 95.0)); // capped
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 61.0, rig.controller.rampedTarget());
}

void test_ramp_down_uses_the_cooling_rate() {
    PeltierControllerConfig config = unitPController();
    config.ramp.maxHeatingRate = 2.0;
    config.ramp.maxCoolingRate = 1.0;
    Rig rig(config);
    rig.controller.start(0.0, 95.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.25, rig.controller.update(kDt, 95.0, 60.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 94.75, rig.controller.rampedTarget());
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.25F, rig.cool.duty());
}

void test_ramp_restarts_from_the_block_temperature_on_start() {
    PeltierControllerConfig config = unitPController();
    config.ramp.maxHeatingRate = 2.0;
    config.ramp.maxCoolingRate = 2.0;
    Rig rig(config);
    rig.controller.start(0.0, 60.0);
    rig.controller.update(kDt, 60.0, 95.0);
    rig.controller.stop();
    rig.controller.start(100.0, 80.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 80.0, rig.controller.rampedTarget());
    rig.controller.update(100.0 + kDt, 80.0, 95.0);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 80.5, rig.controller.rampedTarget());
}

// --- PeltierController: reversal through zero passes through the drive's dead time -------------

void test_reversal_through_zero_waits_the_dead_time_then_cools() {
    PeltierDriveConfig driveConfig;
    driveConfig.deadTimeMs = kDeadTimeMs;
    Rig rig(unitPController(), driveConfig);
    rig.controller.start(0.0, 60.0);

    // Heating: block 0.5 C below target.
    rig.controller.update(kDt, 60.0, 60.5);
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.5F, rig.heat.duty());

    // The output crosses zero: target meets the block, the bridge is released.
    rig.clock.advance(10);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, rig.controller.update(2 * kDt, 60.0, 60.0));
    TEST_ASSERT_TRUE(rig.isSafe());

    // Then negative inside the dead time: the drive holds the cool command, pins stay safe.
    rig.clock.advance(10);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.5, rig.controller.update(3 * kDt, 60.0, 59.5));
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_TRUE(rig.drive.inDeadTime());
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, -0.5F, rig.drive.commanded());

    // Still inside the dead time one sample later: it counts from the release, 10 ms in.
    rig.clock.advance(kDeadTimeMs - 11);
    rig.controller.update(4 * kDt, 60.0, 59.5);
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_TRUE(rig.drive.inDeadTime());

    // Dead time over: the cool side drives with the controller's magnitude.
    rig.clock.advance(1);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.5, rig.controller.update(5 * kDt, 60.0, 59.5));
    TEST_ASSERT_FALSE(rig.drive.inDeadTime());
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.0F, rig.heat.duty());
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.5F, rig.cool.duty());
    TEST_ASSERT_TRUE(rig.enable.isOn());
}

void test_direct_sign_flip_between_samples_never_drives_both_sides() {
    PeltierDriveConfig driveConfig;
    driveConfig.deadTimeMs = kDeadTimeMs;
    Rig rig(unitPController(), driveConfig);
    rig.controller.start(0.0, 60.0);
    rig.controller.update(kDt, 60.0, 60.5);
    rig.clock.advance(250);
    rig.controller.update(2 * kDt, 60.0, 59.5); // heat -> cool with no zero sample in between
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_TRUE(rig.drive.inDeadTime());
    rig.clock.advance(kDeadTimeMs);
    rig.controller.update(3 * kDt, 60.0, 59.5);
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.0F, rig.heat.duty());
    TEST_ASSERT_FLOAT_WITHIN(kDutyTol, 0.5F, rig.cool.duty());
}

void test_non_finite_temperature_releases_the_bridge_and_holds_the_ramp() {
    PeltierControllerConfig config = unitPController();
    config.ramp.maxHeatingRate = 2.0;
    config.ramp.maxCoolingRate = 2.0;
    Rig rig(config);
    rig.controller.start(0.0, 60.0);
    rig.controller.update(kDt, 60.0, 95.0);
    TEST_ASSERT_FALSE(rig.isSafe());
    // The safety layer (#5) rejects bad readings upstream; the controller still refuses to act
    // on one rather than let NaN arithmetic pick a duty.
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, rig.controller.update(2 * kDt, std::nan(""), 95.0));
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_TRUE(rig.drive.isEnabled()); // not a fault trip: that is the safety layer's call
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 60.5, rig.controller.rampedTarget());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, rig.controller.update(3 * kDt, kInf, 95.0));
    TEST_ASSERT_TRUE(rig.isSafe());
    // A good reading resumes from where the ramp was held.
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.75, rig.controller.update(4 * kDt, 60.0, 95.0));
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 61.0, rig.controller.rampedTarget());
}

void test_start_with_a_non_finite_temperature_does_not_arm_the_drive() {
    Rig rig(unitPController());
    TEST_ASSERT_FALSE(rig.controller.start(0.0, std::nan("")));
    TEST_ASSERT_FALSE(rig.controller.running());
    TEST_ASSERT_FALSE(rig.drive.isEnabled());
    TEST_ASSERT_TRUE(rig.controller.start(0.0, 60.0));
    TEST_ASSERT_TRUE(rig.controller.running());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pid_default_lower_bound_is_zero_heat_only);
    RUN_TEST(test_pid_negative_bound_gives_negative_output_above_target);
    RUN_TEST(test_pid_output_saturates_at_min_and_max_power);
    RUN_TEST(test_pid_integral_accumulates_negative_down_to_min_power);
    RUN_TEST(test_pid_integral_unwinds_through_zero_on_reversal);
    RUN_TEST(test_pid_invalid_min_power_falls_back_to_heat_only);
    RUN_TEST(test_pid_reset_at_a_time_and_temperature_does_not_wind_the_integral);
    RUN_TEST(test_ramp_first_update_without_reset_adopts_the_target);
    RUN_TEST(test_ramp_rises_no_faster_than_the_heating_rate_and_lands_exactly);
    RUN_TEST(test_ramp_falls_no_faster_than_the_cooling_rate);
    RUN_TEST(test_ramp_step_scales_with_the_sample_interval);
    RUN_TEST(test_ramp_default_is_unlimited);
    RUN_TEST(test_ramp_holds_when_time_does_not_advance);
    RUN_TEST(test_ramp_holds_on_nan_target);
    RUN_TEST(test_ramp_hold_consumes_time_without_moving);
    RUN_TEST(test_ramp_follows_a_target_that_changes_mid_ramp);
    RUN_TEST(test_ramp_reset_restarts_from_the_given_value);
    RUN_TEST(test_ramp_invalid_rates_hold_the_setpoint);
    RUN_TEST(test_controller_commands_nothing_before_start);
    RUN_TEST(test_start_arms_the_drive_without_driving);
    RUN_TEST(test_stop_puts_the_drive_in_its_safe_state_and_ignores_updates);
    RUN_TEST(test_start_late_in_uptime_does_not_wind_the_integral);
    RUN_TEST(test_below_target_heats_with_the_pid_output_as_duty);
    RUN_TEST(test_above_target_cools_with_the_negated_pid_output_as_duty);
    RUN_TEST(test_at_target_releases_the_bridge);
    RUN_TEST(test_output_saturates_at_the_drive_duty_cap_in_both_directions);
    RUN_TEST(test_a_lower_pid_max_power_wins_over_the_drive_cap);
    RUN_TEST(test_output_sign_always_matches_the_side_driving);
    RUN_TEST(test_setpoint_step_is_ramped_and_the_pid_tracks_the_ramp);
    RUN_TEST(test_ramp_down_uses_the_cooling_rate);
    RUN_TEST(test_ramp_restarts_from_the_block_temperature_on_start);
    RUN_TEST(test_reversal_through_zero_waits_the_dead_time_then_cools);
    RUN_TEST(test_direct_sign_flip_between_samples_never_drives_both_sides);
    RUN_TEST(test_non_finite_temperature_releases_the_bridge_and_holds_the_ramp);
    RUN_TEST(test_start_with_a_non_finite_temperature_does_not_arm_the_drive);
    return UNITY_END();
}
