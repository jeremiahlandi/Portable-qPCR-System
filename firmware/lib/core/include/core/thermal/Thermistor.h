#pragma once

// Port of Klipper's thermistor model (klippy/extras/thermistor.py): Steinhart-Hart from
// three calibration points, with a beta fallback. Ported from Klipper, Copyright (C)
// 2016-2019 Kevin O'Connor, GNU GPLv3.

namespace qpcr::thermal {

/// NTC thermistor on a pull-up divider, read as an ADC fraction in [0, 1].
class Thermistor {
public:
    explicit Thermistor(double pullupResistor, double inlineResistor = 0.0)
        : pullup_(pullupResistor), inlineResistor_(inlineResistor) {}

    /// Three (temperature °C, resistance Ω) points. Falls back to a beta model if the
    /// points do not yield a positive cubic coefficient. Returns false on that fallback.
    bool setupCoefficients(double t1, double r1, double t2, double r2, double t3, double r3);
    /// Beta model from one point.
    void setupCoefficientsBeta(double t1, double r1, double beta);

    [[nodiscard]] double temperatureFromAdc(double adcFraction) const;
    [[nodiscard]] double adcFromTemperature(double temp) const;

    [[nodiscard]] double c1() const { return c1_; }
    [[nodiscard]] double c2() const { return c2_; }
    [[nodiscard]] double c3() const { return c3_; }

private:
    double pullup_;
    double inlineResistor_;
    double c1_ = 0.0;
    double c2_ = 0.0;
    double c3_ = 0.0;
};

} // namespace qpcr::thermal
