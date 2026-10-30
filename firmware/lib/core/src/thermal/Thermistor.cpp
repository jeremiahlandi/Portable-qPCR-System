#include "core/thermal/Thermistor.h"

#include <algorithm>
#include <cmath>

#include "core/thermal/KlipperPort.h"

namespace qpcr::thermal {

bool Thermistor::setupCoefficients(double t1, double r1, double t2, double r2, double t3,
                                   double r3) {
    // Three linear equations in c1, c2, c3.
    const double invT1 = 1.0 / (t1 - klipper::kKelvinToCelsius);
    const double invT2 = 1.0 / (t2 - klipper::kKelvinToCelsius);
    const double invT3 = 1.0 / (t3 - klipper::kKelvinToCelsius);
    const double lnR1 = std::log(r1);
    const double lnR2 = std::log(r2);
    const double lnR3 = std::log(r3);
    const double ln3R1 = lnR1 * lnR1 * lnR1;
    const double ln3R2 = lnR2 * lnR2 * lnR2;
    const double ln3R3 = lnR3 * lnR3 * lnR3;

    const double invT12 = invT1 - invT2;
    const double invT13 = invT1 - invT3;
    const double lnR12 = lnR1 - lnR2;
    const double lnR13 = lnR1 - lnR3;
    const double ln3R12 = ln3R1 - ln3R2;
    const double ln3R13 = ln3R1 - ln3R3;

    c3_ = (invT12 - invT13 * lnR12 / lnR13) / (ln3R12 - ln3R13 * lnR12 / lnR13);
    if (c3_ <= 0.0) {
        const double beta = lnR13 / invT13;
        setupCoefficientsBeta(t1, r1, beta);
        return false;
    }
    c2_ = (invT12 - c3_ * ln3R12) / lnR12;
    c1_ = invT1 - c2_ * lnR1 - c3_ * ln3R1;
    return true;
}

void Thermistor::setupCoefficientsBeta(double t1, double r1, double beta) {
    const double invT1 = 1.0 / (t1 - klipper::kKelvinToCelsius);
    const double lnR1 = std::log(r1);
    c3_ = 0.0;
    c2_ = 1.0 / beta;
    c1_ = invT1 - c2_ * lnR1;
}

double Thermistor::temperatureFromAdc(double adcFraction) const {
    const double adc = std::max(0.00001, std::min(0.99999, adcFraction));
    const double r = pullup_ * adc / (1.0 - adc);
    const double lnR = std::log(r - inlineResistor_);
    const double invT = c1_ + c2_ * lnR + c3_ * lnR * lnR * lnR;
    return 1.0 / invT + klipper::kKelvinToCelsius;
}

double Thermistor::adcFromTemperature(double temp) const {
    if (temp <= klipper::kKelvinToCelsius) {
        return 1.0;
    }
    const double invT = 1.0 / (temp - klipper::kKelvinToCelsius);
    double lnR = 0.0;
    if (c3_ != 0.0) {
        // Cardano's formula for the cubic in ln(r).
        const double y = (c1_ - invT) / (2.0 * c3_);
        const double q = c2_ / (3.0 * c3_);
        const double x = std::sqrt(q * q * q + y * y);
        lnR = std::cbrt(x - y) - std::cbrt(x + y);
    } else {
        lnR = (invT - c1_) / c2_;
    }
    const double r = std::exp(lnR) + inlineResistor_;
    return r / (pullup_ + r);
}

} // namespace qpcr::thermal
