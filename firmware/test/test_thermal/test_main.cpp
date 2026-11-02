// Replay tests for the Klipper thermal port (issue #4). The expected files are produced by
// the unmodified Klipper Python classes via firmware/tools/klipper_reference/replay.py.
#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "core/thermal/HeaterCheck.h"
#include "core/thermal/Max31865.h"
#include "core/thermal/Pid.h"
#include "core/thermal/PidAutotune.h"
#include "core/thermal/TemperatureSmoother.h"
#include "core/thermal/Thermistor.h"

using namespace qpcr::thermal;

namespace {

#ifndef QPCR_PROJECT_DIR
#define QPCR_PROJECT_DIR "."
#endif
const std::string kData = std::string(QPCR_PROJECT_DIR) + "/firmware/test/data/";

// Parameters shared with replay.py; keep in sync.
const PidConfig kPidConfig{30.0, 1.5, 100.0, 1.0, 1.0};
const HeaterCheckConfig kCheckConfig{5.0, 120.0, 2.0, 20.0};
constexpr double kThermistorPullup = 4700.0;
constexpr double kRtdNominal = 1000.0;
constexpr double kRtdReference = 4300.0;
constexpr double kAutotuneTarget = 95.0;
constexpr double kDt = 0.25;

using Row = std::vector<double>;

std::vector<Row> readCsv(const std::string& name) {
    std::vector<Row> rows;
    FILE* f = std::fopen((kData + name).c_str(), "r");
    if (f == nullptr) {
        return rows;
    }
    char line[256];
    bool header = true;
    while (std::fgets(line, sizeof line, f) != nullptr) {
        if (header) {
            header = false;
            continue;
        }
        Row row;
        char* save = nullptr;
        for (const char* tok = strtok_r(line, ",\n", &save); tok != nullptr;
             tok = strtok_r(nullptr, ",\n", &save)) {
            row.push_back(std::strtod(tok, nullptr));
        }
        if (!row.empty()) {
            rows.push_back(row);
        }
    }
    std::fclose(f);
    return rows;
}

std::string readFile(const std::string& name) {
    FILE* f = std::fopen((kData + name).c_str(), "r");
    if (f == nullptr) {
        return {};
    }
    std::string out;
    char buf[512];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) {
        out.append(buf, n);
    }
    std::fclose(f);
    return out;
}

double jsonNumber(const std::string& json, const char* key) {
    const auto pos = json.find(std::string("\"") + key + "\"");
    TEST_ASSERT_MESSAGE(pos != std::string::npos, key);
    const auto colon = json.find(':', pos);
    return std::strtod(json.c_str() + colon + 1, nullptr);
}

// Same plant as replay.py's Plant: first-order block with a lagged heater.
struct Plant {
    double gain = 0.8;
    double tau = 120.0;
    double heaterLag = 5.0;
    double ambient = 25.0;
    double temp = 25.0;
    double power = 0.0;

    double step(double pwm, double dt) {
        power += (pwm - power) * (dt / heaterLag);
        temp += dt * (gain * power - (temp - ambient) / tau);
        return temp;
    }
};

const char* const kTraces[] = {"ramp_hold_steps", "disturbance_at_hold", "runaway_no_heat",
                               "weak_heater"};

void replayPid(const char* trace) {
    const auto rows = readCsv(std::string("traces/") + trace + ".csv");
    const auto expected = readCsv(std::string("expected/") + trace + ".pid.csv");
    TEST_ASSERT_MESSAGE(!rows.empty(), trace);
    TEST_ASSERT_EQUAL_MESSAGE(rows.size(), expected.size(), trace);
    Pid pid(kPidConfig);
    for (size_t i = 0; i < rows.size(); ++i) {
        const double pwm = pid.update(rows[i][0], rows[i][1], rows[i][2]);
        char msg[96];
        std::snprintf(msg, sizeof msg, "%s t=%.2f", trace, rows[i][0]);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-4, expected[i][1], pwm, msg);
    }
}

void replayCheck(const char* trace) {
    const auto rows = readCsv(std::string("traces/") + trace + ".csv");
    const auto expected = readCsv(std::string("expected/") + trace + ".check.csv");
    TEST_ASSERT_MESSAGE(!expected.empty(), trace);
    HeaterCheck check(kCheckConfig);
    size_t e = 0;
    for (const auto& row : rows) {
        const double t = row[0];
        if (std::fabs(t - std::round(t)) > 1e-9) {
            continue; // the check runs once per second
        }
        const auto status = check.check(t, row[1], row[2]);
        char msg[96];
        std::snprintf(msg, sizeof msg, "%s t=%.0f", trace, t);
        TEST_ASSERT_LESS_THAN_MESSAGE(expected.size(), e, msg);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(expected[e][1]),
                                  status == HeaterCheck::Status::Fault ? 1 : 0, msg);
        ++e;
    }
    TEST_ASSERT_EQUAL_MESSAGE(expected.size(), e, trace);
}

} // namespace

void setUp() {}
void tearDown() {}

void test_pid_matches_klipper_on_every_trace() {
    for (const char* trace : kTraces) {
        replayPid(trace);
    }
}

void test_heater_check_matches_klipper_on_every_trace() {
    for (const char* trace : kTraces) {
        replayCheck(trace);
    }
}

void test_heater_check_trips_on_runaway_and_stays_tripped() {
    const auto expected = readCsv("expected/runaway_no_heat.check.csv");
    TEST_ASSERT_EQUAL(1, static_cast<int>(expected.back()[1]));
    HeaterCheck check(kCheckConfig);
    bool tripped = false;
    for (double t = 0.0; t < 200.0; t += 1.0) {
        if (check.check(t, 25.0, 95.0) == HeaterCheck::Status::Fault) {
            tripped = true;
        }
    }
    TEST_ASSERT_TRUE(tripped);
    TEST_ASSERT_TRUE(check.faulted());
    // Sticky: even at target it stays faulted until reset.
    TEST_ASSERT_EQUAL(static_cast<int>(HeaterCheck::Status::Fault),
                      static_cast<int>(check.check(300.0, 95.0, 95.0)));
    check.reset();
    TEST_ASSERT_EQUAL(static_cast<int>(HeaterCheck::Status::Ok),
                      static_cast<int>(check.check(301.0, 95.0, 95.0)));
}

void test_thermistor_matches_klipper() {
    Thermistor th(kThermistorPullup);
    TEST_ASSERT_TRUE(th.setupCoefficients(25.0, 100000.0, 150.0, 1641.9, 250.0, 226.15));
    const auto coeffs = readFile("expected/thermistor_coefficients.json");
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, jsonNumber(coeffs, "c1"), th.c1());
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, jsonNumber(coeffs, "c2"), th.c2());
    TEST_ASSERT_DOUBLE_WITHIN(1e-14, jsonNumber(coeffs, "c3"), th.c3());
    for (const auto& row : readCsv("expected/thermistor_adc_to_temp.csv")) {
        TEST_ASSERT_DOUBLE_WITHIN(0.01, row[1], th.temperatureFromAdc(row[0]));
    }
    for (const auto& row : readCsv("expected/thermistor_temp_to_adc.csv")) {
        TEST_ASSERT_DOUBLE_WITHIN(1e-6, row[1],
                                  th.adcFromTemperature(row[0])); // CSV has 6 decimals
    }
}

void test_max31865_matches_klipper() {
    Max31865Conversion conv(kRtdNominal, kRtdReference);
    const auto rows = readCsv("expected/max31865.csv");
    TEST_ASSERT_FALSE(rows.empty());
    for (const auto& row : rows) {
        TEST_ASSERT_EQUAL_UINT16(static_cast<unsigned>(row[1]), conv.adcFromTemperature(row[0]));
        TEST_ASSERT_DOUBLE_WITHIN(0.01, row[2],
                                  conv.temperatureFromAdc(static_cast<std::uint16_t>(row[1])));
    }
}

void test_autotune_matches_klipper() {
    const auto expected = readFile("expected/autotune.json");
    TEST_ASSERT_FALSE(expected.empty());
    PidAutotune tune(kAutotuneTarget, 1.0);
    Plant plant;
    double t = 0.0;
    int steps = 0;
    while (tune.busy() && steps < 40000) {
        const double pwm = tune.update(t, plant.temp);
        plant.step(pwm, kDt);
        t = std::round((t + kDt) * 1e6) / 1e6;
        ++steps;
    }
    TEST_ASSERT_FALSE(tune.busy());
    TEST_ASSERT_EQUAL(static_cast<int>(jsonNumber(expected, "steps")), steps);
    TEST_ASSERT_EQUAL(static_cast<int>(jsonNumber(expected, "peaks")), tune.peakCount());
    const PidGains gains = tune.finalGains();
    const double kp = jsonNumber(expected, "kp");
    const double ki = jsonNumber(expected, "ki");
    const double kd = jsonNumber(expected, "kd");
    TEST_ASSERT_DOUBLE_WITHIN(0.01 * kp, kp, gains.kp);
    TEST_ASSERT_DOUBLE_WITHIN(0.01 * ki, ki, gains.ki);
    TEST_ASSERT_DOUBLE_WITHIN(0.01 * kd, kd, gains.kd);
}

void test_smoother_matches_klipper_formula() {
    TemperatureSmoother s(1.0);
    // Klipper: adj = min(dt / smooth_time, 1); smoothed += (temp - smoothed) * adj
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 25.0, s.update(1.0, 25.0)); // dt=1 -> full step
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 25.0 + 0.25 * 75.0, s.update(1.25, 100.0));
}

void test_nan_sample_does_not_crash_and_stays_bounded() {
    Pid pid(kPidConfig);
    pid.update(0.25, 25.0, 95.0);
    const double out = pid.update(0.5, std::nan(""), 95.0);
    TEST_ASSERT_TRUE(out >= 0.0 && out <= kPidConfig.maxPower);
    HeaterCheck check(kCheckConfig);
    (void)check.check(1.0, std::nan(""), 95.0); // the safety layer (#5) rejects NaN upstream
    TEST_ASSERT_FALSE(check.faulted());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pid_matches_klipper_on_every_trace);
    RUN_TEST(test_heater_check_matches_klipper_on_every_trace);
    RUN_TEST(test_heater_check_trips_on_runaway_and_stays_tripped);
    RUN_TEST(test_thermistor_matches_klipper);
    RUN_TEST(test_max31865_matches_klipper);
    RUN_TEST(test_autotune_matches_klipper);
    RUN_TEST(test_smoother_matches_klipper_formula);
    RUN_TEST(test_nan_sample_does_not_crash_and_stays_bounded);
    return UNITY_END();
}
