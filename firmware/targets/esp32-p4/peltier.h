#pragma once

#include "core/thermal/PeltierDrive.h"

// Peltier H-bridge glue for the ESP32-P4 target (issue #7): the IBT_2 (dual BTS7960) on the J6
// header, RPWM / LPWM on LEDC at 20 kHz, 10-bit, and the tied R_EN + L_EN on a GPIO. Pins come
// from pins.h; the wiring is in hardware/PINS.md.

namespace qpcr::target {

constexpr std::uint32_t kHBridgePwmHz = 20000;
constexpr std::uint8_t kHBridgePwmBits = 10;

/// Puts the bridge pins in their safe state (enable low, both PWMs at zero) and constructs the
/// drive, left disabled. Nothing drives the Peltier until a control loop calls
/// peltierDrive().enable() and command(). Returns false if an LEDC channel could not be
/// claimed; the drive then exists but its PWM outputs stay inert.
bool peltierInit();

/// The one drive instance. Valid after peltierInit().
qpcr::thermal::PeltierDrive& peltierDrive();

} // namespace qpcr::target
