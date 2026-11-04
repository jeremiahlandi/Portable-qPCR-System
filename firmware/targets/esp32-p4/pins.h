#pragma once

// Pin allocation for the Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5 development board (issue #2,
// LI-DL-004, revised 2026-10-30). Single source of truth is hardware/PINS.md; keep both in sync.
//
// Display and touch are routed by the board itself. GPIO numbers come from Waveshare's own
// Arduino board library (examples/arduino/libraries/Waveshare_LCD35/src/lcd35_board.h in
// github.com/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-3.5). Instrument signals on the free header
// are assigned by their own issues and are still to be filled in (issue #16).

namespace qpcr::pins {

// Display: ST7796 320x480 IPS over SPI (board-routed). Vendor runs the bus at 80 MHz.
constexpr int kDisplaySck = 21;
constexpr int kDisplayMosi = 20;
constexpr int kDisplayMiso = -1; // schematic routes LCD_MISO to GPIO22; unused (panel not read)
constexpr int kDisplayCs = 23;
constexpr int kDisplayDc = 26;
constexpr int kDisplayRst = 27;
constexpr int kDisplayBacklight = 28; // LEDC PWM, 5 kHz

// Board I2C bus (FT6336 touch, AXP2101 PMIC, ES8311 codec; also exposed on the header).
constexpr int kI2cSda = 7;
constexpr int kI2cScl = 8;

// FT6336 touch controller (schematic J14): reset on GPIO29 with a 10 k pull-down, so the chip
// stays in reset until the firmware drives it high; interrupt on GPIO50 (4.7 k pull-up), unused.
constexpr int kTouchRst = 29;
constexpr int kTouchInt = 50;

// TF card slot, SDIO 4-bit (issue #14).
constexpr int kSdClk = 43;
constexpr int kSdCmd = 44;
constexpr int kSdD0 = 39;
constexpr int kSdD1 = 40;
constexpr int kSdD2 = 41;
constexpr int kSdD3 = 42;

// Audio codec ES8311 (I2S) and NS4150B amplifier enable (issue #13 may reuse for the buzzer).
constexpr int kI2sMclk = 13;
constexpr int kI2sBclk = 12;
constexpr int kI2sLrck = 10;
constexpr int kI2sDout = 9; // P4 -> codec
constexpr int kI2sDin = 11; // codec -> P4
constexpr int kAudioPaEnable = 53;

// Peltier H-bridge, IBT_2 (dual BTS7960) on the J6 header (issue #7). Assigned 2026-11-01, not
// yet driven by firmware. R_EN and L_EN are wired together to kHBridgeEnable; logic VCC is 3V3.
constexpr int kHBridgePwmHeat = 4;    // RPWM, LEDC
constexpr int kHBridgePwmCool = 5;    // LPWM, LEDC
constexpr int kHBridgeEnable = 3;     // R_EN + L_EN
constexpr int kHBridgeSenseHeat = 51; // R_IS, ADC2 (optional)
constexpr int kHBridgeSenseCool = 52; // L_IS, ADC2 (optional)

} // namespace qpcr::pins
