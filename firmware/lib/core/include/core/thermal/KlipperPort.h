#pragma once

// Constants shared by the modules ported from Klipper (klippy/extras/heaters.py).
// Ported from Klipper, Copyright (C) 2016-2025 Kevin O'Connor, GNU GPLv3.
// See NOTICE at the repository root.

namespace qpcr::thermal::klipper {

constexpr double kKelvinToCelsius = -273.15;
constexpr double kAmbientTemp = 25.0;
/// Klipper expresses PID gains scaled by 255 in its config files; the controller divides by this.
constexpr double kPidParamBase = 255.0;
/// A target of this or below means "off" (matches Klipper's `target_temp <= 0.`).
constexpr double kOffTarget = 0.0;

} // namespace qpcr::thermal::klipper
