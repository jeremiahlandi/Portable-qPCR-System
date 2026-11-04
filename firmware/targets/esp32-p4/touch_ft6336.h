#pragma once

// FT6336 capacitive touch controller on the board I2C bus (issue #11). Polled on LVGL's input
// timer. The board pulls its reset line (GPIO29) low, so touchInit() releases it first; the
// interrupt line (GPIO50) is not used. See hardware/PINS.md.

namespace qpcr::target {

/// Starts the I2C bus and probes the controller. Returns false if it does not answer.
bool touchInit();

/// Registers the controller as LVGL's pointer device. Call after displayInit().
void touchAttachToLvgl();

} // namespace qpcr::target
