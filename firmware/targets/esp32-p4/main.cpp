#include <Arduino.h>
#include <algorithm>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "core/Version.h"

#include "pins.h"

namespace {

// Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5: ST7796 320x480 over SPI, routed by the board.
// IPS panel (inverted colours) as in Waveshare's Arduino_GFX board library. Their "rotation 4"
// maps to LovyanGFX offset_rotation 0: with 4 the image came out X-mirrored (verified 2026-11-01).
class LGFX_P4 : public lgfx::LGFX_Device {
    lgfx::Panel_ST7796 _panel_instance;
    lgfx::Bus_SPI _bus_instance;

public:
    LGFX_P4() {
        auto cfg = _bus_instance.config();
        cfg.spi_host = SPI2_HOST;
        cfg.spi_mode = 0;
        cfg.freq_write = 40000000; // vendor uses 80 MHz; start conservative
        cfg.freq_read = 16000000;
        cfg.pin_sclk = qpcr::pins::kDisplaySck;
        cfg.pin_mosi = qpcr::pins::kDisplayMosi;
        cfg.pin_miso = qpcr::pins::kDisplayMiso;
        cfg.pin_dc = qpcr::pins::kDisplayDc;
        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);

        auto p_cfg = _panel_instance.config();
        p_cfg.pin_cs = qpcr::pins::kDisplayCs;
        p_cfg.pin_rst = qpcr::pins::kDisplayRst;
        p_cfg.pin_busy = -1;
        p_cfg.panel_width = 320;
        p_cfg.panel_height = 480;
        p_cfg.offset_rotation = 0;
        p_cfg.invert = true;
        p_cfg.rgb_order = false;
        p_cfg.readable = false;
        p_cfg.bus_shared = false;
        _panel_instance.config(p_cfg);
        setPanel(&_panel_instance);
    }
};

LGFX_P4 lcd;

// Backlight: LEDC PWM at 5 kHz, 10-bit, as in the vendor library.
void setBacklight(uint8_t percent) {
    static bool attached = false;
    if (!attached) {
        attached = ledcAttach(qpcr::pins::kDisplayBacklight, 5000, 10);
    }
    if (attached) {
        ledcWrite(qpcr::pins::kDisplayBacklight, (1023U * std::min<uint8_t>(percent, 100)) / 100U);
    }
}

void drawBringUpScreen() {
    lcd.setRotation(1); // landscape, 480x320
    lcd.fillScreen(TFT_BLACK);

    lcd.setTextColor(TFT_WHITE);
    lcd.setTextSize(3);
    lcd.setCursor(20, 20);
    lcd.print("SYSTEM VERIFIED");

    lcd.setTextColor(TFT_GREEN);
    lcd.setCursor(20, 80);
    lcd.print("Brain: ONLINE");

    lcd.setTextColor(TFT_BLUE);
    lcd.setCursor(20, 140);
    lcd.print("Screen: ST7796");

    lcd.setTextColor(TFT_WHITE);
    lcd.setTextSize(2);
    lcd.setCursor(20, 220);
    lcd.print("FW ");
    lcd.print(qpcr::firmwareVersion());

    // Colour-order and orientation check: R, G, B bars along the bottom, marker at the origin.
    lcd.fillRect(20, 260, 140, 40, TFT_RED);
    lcd.fillRect(170, 260, 140, 40, TFT_GREEN);
    lcd.fillRect(320, 260, 140, 40, TFT_BLUE);
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_BLACK);
    lcd.setCursor(24, 276);
    lcd.print("RED");
    lcd.setCursor(174, 276);
    lcd.print("GREEN");
    lcd.setCursor(324, 276);
    lcd.print("BLUE");
    lcd.fillTriangle(0, 0, 16, 0, 0, 16, TFT_YELLOW); // top-left origin marker
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.print("Portable qPCR firmware ");
    Serial.println(qpcr::firmwareVersion());

    setBacklight(0);
    const bool ok = lcd.init();
    Serial.print("Display init: ");
    Serial.println(ok ? "OK (ST7796)" : "FAILED");
    if (ok) {
        drawBringUpScreen();
        setBacklight(100);
    }
}

void loop() {
    delay(1000);
}
