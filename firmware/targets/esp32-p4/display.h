#pragma once

#include <cstdint>

// Board display for the Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5 (issue #31) wrapped for LVGL
// (issue #11): LovyanGFX drives the ST7796; LVGL renders into two partial buffers and hands
// them to the flush callback here.

namespace qpcr::target {

constexpr int kScreenWidth = 480; // landscape (LovyanGFX rotation 1)
constexpr int kScreenHeight = 320;

/// Initialises the panel, the backlight (off) and the LVGL display object. Returns false if the
/// panel does not answer; the firmware then runs without a UI (acceptance criterion 4 of #11).
bool displayInit();

/// Backlight 0..100 %. Safe to call before displayInit().
void displaySetBacklight(std::uint8_t percent);

} // namespace qpcr::target
