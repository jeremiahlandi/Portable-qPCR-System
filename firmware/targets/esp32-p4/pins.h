#pragma once

// Pin allocation for the Waveshare ESP32-P4 module dev kit (issue #2, LI-DL-004).
// Verified on hardware: display SPI (PINS.md, "SAVE POINT" commit).
// Everything else is assigned by its own issue and must be checked against the board's
// reserved pins (C6 SDIO, Ethernet, SD, audio, USB) before wiring (issue #16).

namespace qpcr::pins {

// Display: ILI9488 480x320 on SPI3 (verified at 20 MHz)
constexpr int kDisplaySck = 24;
constexpr int kDisplayMosi = 23;
constexpr int kDisplayDc = 22;
constexpr int kDisplayRst = 21;
constexpr int kDisplayCs = 20;

}  // namespace qpcr::pins
