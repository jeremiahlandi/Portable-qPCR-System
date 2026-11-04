#pragma once

// FT6336 capacitive touch controller on the board I2C bus (issue #11). Polled; the controller's
// INT and RST lines are not in Waveshare's vendor header (see hardware/PINS.md), so the chip is
// assumed to be held out of reset by the board and is read on LVGL's input timer.

namespace qpcr::target {

/// Starts the I2C bus and probes the controller. Returns false if it does not answer.
bool touchInit();

/// Registers the controller as LVGL's pointer device. Call after displayInit().
void touchAttachToLvgl();

} // namespace qpcr::target
