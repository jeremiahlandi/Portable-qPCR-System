#include "core/thermal/PidAutotune.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <utility>

#include "core/thermal/KlipperPort.h"

namespace qpcr::thermal {

namespace {
constexpr double kTunePidDelta = 2.5;
constexpr std::size_t kMinPeaks = 12;
constexpr double kPi = 3.14159265358979323846;
} // namespace

PidAutotune::PidAutotune(double calibrateTemp, double maxPower)
    : maxPower_(maxPower), calibrateTemp_(calibrateTemp), activeTarget_(calibrateTemp) {}

double PidAutotune::update(double readTime, double temp) {
    const double target = activeTarget_;
    // Relay: flip the heater when the temperature crosses the (alternating) target.
    if (heating_ && temp >= target) {
        heating_ = false;
        checkPeaks();
        activeTarget_ = calibrateTemp_ - kTunePidDelta;
    } else if (!heating_ && temp <= target) {
        heating_ = true;
        checkPeaks();
        activeTarget_ = calibrateTemp_;
    }
    // Track the extreme of the current half-cycle.
    if (heating_) {
        if (temp < peak_) {
            peak_ = temp;
            peakTime_ = readTime;
        }
        return maxPower_;
    }
    if (temp > peak_) {
        peak_ = temp;
        peakTime_ = readTime;
    }
    return 0.0;
}

bool PidAutotune::busy() const {
    return heating_ || peakCount_ < kMinPeaks;
}

void PidAutotune::checkPeaks() {
    if (peakCount_ < kMaxPeaks) {
        peaks_.at(peakCount_++) = {peakTime_, peak_};
    }
    peak_ = heating_ ? 9999999.0 : -9999999.0;
}

PidGains PidAutotune::calcPid(std::size_t pos) const {
    const double tempDiff = peaks_.at(pos).temp - peaks_.at(pos - 1).temp;
    const double timeDiff = peaks_.at(pos).time - peaks_.at(pos - 2).time;
    // Åström-Hägglund estimate of the ultimate gain and period.
    const double amplitude = 0.5 * std::fabs(tempDiff);
    const double powerAmplitude = 0.5 * maxPower_;
    const double ku = 4.0 * powerAmplitude / (kPi * amplitude);
    const double tu = timeDiff;
    // Ziegler-Nichols.
    const double ti = 0.5 * tu;
    const double td = 0.125 * tu;
    PidGains gains;
    gains.kp = 0.6 * ku * klipper::kPidParamBase;
    gains.ki = gains.kp / ti;
    gains.kd = gains.kp * td;
    return gains;
}

PidGains PidAutotune::finalGains() const {
    if (peakCount_ < 5) {
        return {};
    }
    // Median oscillation period over the recorded peaks (ties broken by position, as the
    // original's tuple sort does).
    std::array<std::pair<double, std::size_t>, kMaxPeaks> cycleTimes{};
    std::size_t n = 0;
    for (std::size_t pos = 4; pos < peakCount_; ++pos) {
        cycleTimes.at(n++) = {peaks_.at(pos).time - peaks_.at(pos - 2).time, pos};
    }
    std::sort(cycleTimes.begin(), std::next(cycleTimes.begin(), static_cast<long>(n)));
    return calcPid(cycleTimes.at(n / 2).second);
}

} // namespace qpcr::thermal
