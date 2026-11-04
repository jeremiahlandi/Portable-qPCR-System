// Host tests for the bidirectional Peltier H-bridge drive (issue #7): sign-to-side mapping,
// the duty cap, dead time on reversal and the disabled safe state, all through the fakes.
#include <unity.h>

#include <cmath>
#include <cstdint>

#include "core/thermal/PeltierDrive.h"
#include "fakes/FakeClock.h"
#include "fakes/FakeDigitalOutput.h"
#include "fakes/FakePwmOutput.h"

using namespace qpcr;
using thermal::PeltierDrive;
using thermal::PeltierDriveConfig;

namespace {

constexpr float kTol = 0.0005F;
constexpr std::uint32_t kDeadTimeMs = 50;

struct Rig {
    fakes::FakePwmOutput heat;
    fakes::FakePwmOutput cool;
    fakes::FakeDigitalOutput enable;
    fakes::FakeClock clock;
    PeltierDrive drive;

    explicit Rig(const PeltierDriveConfig& config = {})
        : drive(heat, cool, enable, clock, config) {}

    /// The bridge is in its safe state: both PWMs at zero and the enable pin low.
    [[nodiscard]] bool isSafe() const {
        return heat.duty() == 0.0F && cool.duty() == 0.0F && !enable.isOn();
    }
};

PeltierDriveConfig deadTimeConfig() {
    PeltierDriveConfig config;
    config.deadTimeMs = kDeadTimeMs;
    return config;
}

} // namespace

void setUp() {}
void tearDown() {}

// --- construction and the disabled safe state -------------------------------------------------

void test_constructed_drive_is_disabled_and_safe() {
    Rig rig;
    TEST_ASSERT_FALSE(rig.drive.isEnabled());
    TEST_ASSERT_TRUE(rig.isSafe());
    // The constructor writes the safe state to the pins rather than trusting their reset value.
    TEST_ASSERT_EQUAL(1U, rig.heat.setCount);
    TEST_ASSERT_EQUAL(1U, rig.cool.setCount);
    TEST_ASSERT_EQUAL(1U, rig.enable.setCount);
    TEST_ASSERT_EQUAL(static_cast<int>(PeltierDrive::Side::Off),
                      static_cast<int>(rig.drive.side()));
}

void test_command_while_disabled_drives_nothing() {
    Rig rig;
    rig.drive.command(0.5F);
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0F, rig.drive.applied());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.5F, rig.drive.commanded());
}

void test_enable_then_command_drives() {
    Rig rig;
    rig.drive.command(0.5F);
    rig.drive.enable();
    TEST_ASSERT_TRUE(rig.drive.isEnabled());
    // Enabling alone must not start driving a command that was issued while disabled.
    TEST_ASSERT_TRUE(rig.isSafe());
    rig.drive.command(0.5F);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.5F, rig.heat.duty());
    TEST_ASSERT_TRUE(rig.enable.isOn());
}

// --- sign-to-side mapping ----------------------------------------------------------------------

void test_positive_command_heats() {
    Rig rig;
    rig.drive.enable();
    rig.drive.command(0.4F);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.4F, rig.heat.duty());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0F, rig.cool.duty());
    TEST_ASSERT_TRUE(rig.enable.isOn());
    TEST_ASSERT_EQUAL(static_cast<int>(PeltierDrive::Side::Heat),
                      static_cast<int>(rig.drive.side()));
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.4F, rig.drive.applied());
}

void test_negative_command_cools() {
    Rig rig;
    rig.drive.enable();
    rig.drive.command(-0.3F);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0F, rig.heat.duty());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.3F, rig.cool.duty());
    TEST_ASSERT_TRUE(rig.enable.isOn());
    TEST_ASSERT_EQUAL(static_cast<int>(PeltierDrive::Side::Cool),
                      static_cast<int>(rig.drive.side()));
    TEST_ASSERT_FLOAT_WITHIN(kTol, -0.3F, rig.drive.applied());
}

void test_zero_command_releases_the_bridge() {
    Rig rig;
    rig.drive.enable();
    rig.drive.command(0.4F);
    rig.drive.command(0.0F);
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_EQUAL(static_cast<int>(PeltierDrive::Side::Off),
                      static_cast<int>(rig.drive.side()));
    TEST_ASSERT_TRUE(rig.drive.isEnabled());
}

void test_nan_command_is_treated_as_zero() {
    Rig rig;
    rig.drive.enable();
    rig.drive.command(0.4F);
    rig.drive.command(std::nanf(""));
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0F, rig.drive.commanded());
}

// --- duty cap ----------------------------------------------------------------------------------

void test_default_cap_keeps_cp303385h_under_three_amps_at_15v() {
    // CUI CP303385H: 3.0 A max, about 4 ohm. 15 V x cap / 4 ohm must stay below 3 A.
    constexpr float kSupplyVolts = 15.0F;
    constexpr float kPeltierOhms = 4.0F;
    const PeltierDriveConfig config;
    TEST_ASSERT_TRUE(kSupplyVolts * config.maxDuty / kPeltierOhms < 3.0F);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.75F, config.maxDuty);
}

void test_full_scale_command_is_capped_on_both_sides() {
    Rig rig;
    rig.drive.enable();
    rig.drive.command(1.0F);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.75F, rig.heat.duty());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.75F, rig.drive.applied());
    rig.drive.command(0.0F);
    rig.clock.advance(1000);
    rig.drive.command(-5.0F); // out of range: clamped to -1 then capped
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.75F, rig.cool.duty());
    TEST_ASSERT_FLOAT_WITHIN(kTol, -0.75F, rig.drive.applied());
    TEST_ASSERT_FLOAT_WITHIN(kTol, -1.0F, rig.drive.commanded());
}

void test_commands_below_the_cap_pass_through_unscaled() {
    Rig rig;
    rig.drive.enable();
    rig.drive.command(0.6F);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.6F, rig.heat.duty());
}

void test_custom_cap_is_honoured_and_clamped_to_unit_range() {
    PeltierDriveConfig low;
    low.maxDuty = 0.2F;
    Rig rigLow(low);
    rigLow.drive.enable();
    rigLow.drive.command(0.9F);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.2F, rigLow.heat.duty());

    PeltierDriveConfig high;
    high.maxDuty = 1.5F;
    Rig rigHigh(high);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0F, rigHigh.drive.config().maxDuty);

    PeltierDriveConfig negative;
    negative.maxDuty = -0.5F;
    Rig rigNeg(negative);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0F, rigNeg.drive.config().maxDuty);
    rigNeg.drive.enable();
    rigNeg.drive.command(1.0F);
    TEST_ASSERT_TRUE(rigNeg.isSafe());
}

// --- dead time on reversal ---------------------------------------------------------------------

void test_reversal_first_releases_the_bridge() {
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.command(-0.5F);
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_TRUE(rig.drive.inDeadTime());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0F, rig.drive.applied());
    TEST_ASSERT_FLOAT_WITHIN(kTol, -0.5F, rig.drive.commanded());
}

void test_reversal_waits_the_full_dead_time() {
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.command(-0.5F);
    rig.clock.advance(kDeadTimeMs - 1);
    rig.drive.tick();
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_TRUE(rig.drive.inDeadTime());
    rig.clock.advance(1);
    rig.drive.tick();
    TEST_ASSERT_FALSE(rig.drive.inDeadTime());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0F, rig.heat.duty());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.5F, rig.cool.duty());
    TEST_ASSERT_TRUE(rig.enable.isOn());
    TEST_ASSERT_EQUAL(static_cast<int>(PeltierDrive::Side::Cool),
                      static_cast<int>(rig.drive.side()));
}

void test_repeated_commands_during_dead_time_stay_off_and_latest_wins() {
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.command(-0.5F);
    rig.clock.advance(10);
    rig.drive.command(-0.2F);
    TEST_ASSERT_TRUE(rig.isSafe());
    rig.clock.advance(kDeadTimeMs);
    rig.drive.command(-0.7F);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.7F, rig.cool.duty());
}

void test_reversal_through_zero_still_waits() {
    // Heat, release, then cool a few ms later: the dead time counts from the release.
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.clock.advance(5);
    rig.drive.command(0.0F);
    rig.clock.advance(kDeadTimeMs / 2);
    rig.drive.command(-0.5F);
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_TRUE(rig.drive.inDeadTime());
    rig.clock.advance(kDeadTimeMs / 2);
    rig.drive.tick();
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.5F, rig.cool.duty());
}

void test_reversal_after_a_long_idle_is_immediate() {
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.command(0.0F);
    rig.clock.advance(kDeadTimeMs);
    rig.drive.command(-0.5F);
    TEST_ASSERT_FALSE(rig.drive.inDeadTime());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.5F, rig.cool.duty());
}

void test_same_side_recommand_needs_no_dead_time() {
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.command(0.0F);
    rig.drive.command(0.3F);
    TEST_ASSERT_FALSE(rig.drive.inDeadTime());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.3F, rig.heat.duty());
    rig.drive.command(0.6F); // change of magnitude on the same side
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.6F, rig.heat.duty());
    TEST_ASSERT_TRUE(rig.enable.isOn());
}

void test_abandoned_reversal_returns_to_the_original_side_immediately() {
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.command(-0.5F);
    rig.clock.advance(10);
    rig.drive.command(0.4F); // changed its mind: same side as before the reversal
    TEST_ASSERT_FALSE(rig.drive.inDeadTime());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.4F, rig.heat.duty());
}

void test_dead_time_survives_clock_wrap() {
    Rig rig(deadTimeConfig());
    rig.clock.now = 0xFFFFFFFFU - 10U;
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.command(-0.5F);
    rig.clock.advance(kDeadTimeMs - 1); // wraps past zero
    rig.drive.tick();
    TEST_ASSERT_TRUE(rig.drive.inDeadTime());
    rig.clock.advance(1);
    rig.drive.tick();
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.5F, rig.cool.duty());
}

void test_zero_dead_time_reverses_immediately() {
    PeltierDriveConfig config;
    config.deadTimeMs = 0;
    Rig rig(config);
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.command(-0.5F);
    TEST_ASSERT_FALSE(rig.drive.inDeadTime());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.5F, rig.cool.duty());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0F, rig.heat.duty());
}

// --- disable() ---------------------------------------------------------------------------------

void test_disable_from_active_drive_is_safe_and_sticky() {
    Rig rig;
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.disable();
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_FALSE(rig.drive.isEnabled());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0F, rig.drive.applied());
    rig.drive.command(0.5F);
    rig.drive.tick();
    TEST_ASSERT_TRUE(rig.isSafe());
}

void test_disable_during_dead_time_keeps_the_bridge_off_when_it_elapses() {
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.command(-0.5F);
    rig.drive.disable();
    rig.clock.advance(kDeadTimeMs * 2);
    rig.drive.tick();
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_FALSE(rig.drive.inDeadTime());
}

void test_reenable_after_disable_still_respects_dead_time_for_a_reversal() {
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    rig.drive.command(0.5F);
    rig.drive.disable();
    rig.clock.advance(10);
    rig.drive.enable();
    rig.drive.command(-0.5F);
    TEST_ASSERT_TRUE(rig.isSafe());
    TEST_ASSERT_TRUE(rig.drive.inDeadTime());
    rig.clock.advance(kDeadTimeMs);
    rig.drive.tick();
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.5F, rig.cool.duty());
}

void test_enable_pin_is_only_high_while_one_side_drives() {
    Rig rig(deadTimeConfig());
    rig.drive.enable();
    TEST_ASSERT_FALSE(rig.enable.isOn());
    rig.drive.command(0.5F);
    TEST_ASSERT_TRUE(rig.enable.isOn());
    rig.drive.command(0.0F);
    TEST_ASSERT_FALSE(rig.enable.isOn());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_constructed_drive_is_disabled_and_safe);
    RUN_TEST(test_command_while_disabled_drives_nothing);
    RUN_TEST(test_enable_then_command_drives);
    RUN_TEST(test_positive_command_heats);
    RUN_TEST(test_negative_command_cools);
    RUN_TEST(test_zero_command_releases_the_bridge);
    RUN_TEST(test_nan_command_is_treated_as_zero);
    RUN_TEST(test_default_cap_keeps_cp303385h_under_three_amps_at_15v);
    RUN_TEST(test_full_scale_command_is_capped_on_both_sides);
    RUN_TEST(test_commands_below_the_cap_pass_through_unscaled);
    RUN_TEST(test_custom_cap_is_honoured_and_clamped_to_unit_range);
    RUN_TEST(test_reversal_first_releases_the_bridge);
    RUN_TEST(test_reversal_waits_the_full_dead_time);
    RUN_TEST(test_repeated_commands_during_dead_time_stay_off_and_latest_wins);
    RUN_TEST(test_reversal_through_zero_still_waits);
    RUN_TEST(test_reversal_after_a_long_idle_is_immediate);
    RUN_TEST(test_same_side_recommand_needs_no_dead_time);
    RUN_TEST(test_abandoned_reversal_returns_to_the_original_side_immediately);
    RUN_TEST(test_dead_time_survives_clock_wrap);
    RUN_TEST(test_zero_dead_time_reverses_immediately);
    RUN_TEST(test_disable_from_active_drive_is_safe_and_sticky);
    RUN_TEST(test_disable_during_dead_time_keeps_the_bridge_off_when_it_elapses);
    RUN_TEST(test_reenable_after_disable_still_respects_dead_time_for_a_reversal);
    RUN_TEST(test_enable_pin_is_only_high_while_one_side_drives);
    return UNITY_END();
}
