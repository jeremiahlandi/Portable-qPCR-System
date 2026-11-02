#pragma once

// Port of the MAX31865 RTD conversion in Klipper (klippy/extras/spi_temperature.py):
// Callendar-Van Dusen above 0 °C. Ported from Klipper, Copyright (C) 2016-2025
// Kevin O'Connor, GNU GPLv3.

#include <cstdint>

namespace qpcr::thermal {

/// Converts between MAX31865 RTD register values and temperature.
class Max31865Conversion {
public:
    static constexpr int kAdcMax = 32768;

    /// `rtdNominalR` is the RTD resistance at 0 °C (100 for PT100, 1000 for PT1000);
    /// `rtdReferenceR` is the reference resistor on the board (430 for PT100 boards,
    /// 4300 for PT1000 boards).
    Max31865Conversion(double rtdNominalR, double rtdReferenceR);

    /// Temperature from the 16-bit RTD register (fault bit in bit 0).
    [[nodiscard]] double temperatureFromAdc(std::uint16_t adc) const;
    /// RTD register value (fault bit cleared) for a temperature.
    [[nodiscard]] std::uint16_t adcFromTemperature(double temp) const;

private:
    double adcToResistDivNominal_;
};

} // namespace qpcr::thermal
