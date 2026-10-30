#include <unity.h>

#include "fakes/FakeClock.h"
#include "fakes/FakeDigitalOutput.h"
#include "fakes/FakePwmOutput.h"
#include "fakes/FakeTemperatureSensor.h"

using namespace qpcr;

void setUp() {}
void tearDown() {}

void test_sensor_starts_not_ready() {
    fakes::FakeTemperatureSensor sensor;
    const auto reading = sensor.read();
    TEST_ASSERT_FALSE(reading.valid());
    TEST_ASSERT_EQUAL(static_cast<int>(hal::SensorFault::NotReady), static_cast<int>(reading.fault));
}

void test_sensor_set_and_fault() {
    fakes::FakeTemperatureSensor sensor;
    sensor.set(95.25F);
    TEST_ASSERT_TRUE(sensor.read().valid());
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 95.25F, sensor.read().celsius);
    sensor.fail(hal::SensorFault::Open);
    TEST_ASSERT_FALSE(sensor.read().valid());
    TEST_ASSERT_EQUAL(3U, sensor.readCount);
}

void test_pwm_clamps_duty() {
    fakes::FakePwmOutput pwm;
    pwm.setDuty(1.5F);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 1.0F, pwm.duty());
    pwm.setDuty(-0.2F);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, pwm.duty());
    pwm.setDuty(0.4F);
    pwm.off();
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, pwm.duty());
    TEST_ASSERT_EQUAL(4U, pwm.setCount);
}

void test_digital_output() {
    fakes::FakeDigitalOutput out;
    TEST_ASSERT_FALSE(out.isOn());
    out.set(true);
    TEST_ASSERT_TRUE(out.isOn());
}

void test_clock_elapsed_handles_wrap() {
    fakes::FakeClock clock;
    clock.now = 0xFFFFFFF0U;
    const auto start = clock.millis();
    clock.advance(0x20U);  // wraps past zero
    TEST_ASSERT_EQUAL_UINT32(0x20U, clock.elapsedSince(start));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_sensor_starts_not_ready);
    RUN_TEST(test_sensor_set_and_fault);
    RUN_TEST(test_pwm_clamps_duty);
    RUN_TEST(test_digital_output);
    RUN_TEST(test_clock_elapsed_handles_wrap);
    return UNITY_END();
}
