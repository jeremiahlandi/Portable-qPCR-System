#include "core/thermal/Max31865.h"

#include <algorithm>
#include <cmath>

namespace qpcr::thermal {

namespace {
// Callendar-Van Dusen coefficients for platinum.
constexpr double kCvdA = 3.9083e-3;
constexpr double kCvdB = -5.775e-7;
constexpr double kPlatinumMeltingPoint = 1768.3;
} // namespace

Max31865Conversion::Max31865Conversion(double rtdNominalR, double rtdReferenceR)
    : adcToResistDivNominal_((rtdReferenceR / static_cast<double>(kAdcMax)) / rtdNominalR) {}

double Max31865Conversion::temperatureFromAdc(std::uint16_t adc) const {
    const unsigned value = static_cast<unsigned>(adc) >> 1U; // drop the fault bit
    const double rDivNominal = value * adcToResistDivNominal_;
    // R/R0 = 1 + A*t + B*t^2, solved for t with the quadratic formula.
    const double discriminant = std::sqrt(kCvdA * kCvdA - 4.0 * kCvdB * (1.0 - rDivNominal));
    return (-kCvdA + discriminant) / (2.0 * kCvdB);
}

std::uint16_t Max31865Conversion::adcFromTemperature(double temp) const {
    temp = std::min(temp, kPlatinumMeltingPoint);
    const double rDivNominal = 1.0 + kCvdA * temp + kCvdB * temp * temp;
    int adc = static_cast<int>(std::lround(rDivNominal / adcToResistDivNominal_));
    adc = std::max(0, std::min(kAdcMax - 1, adc));
    return static_cast<std::uint16_t>(adc << 1); // room for the fault bit
}

} // namespace qpcr::thermal
