#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "core/Version.h"
#include "core/ui/VersionBadge.h"

#include "pins.h"

class LGFX_P4 : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9488 _panel_instance;
    lgfx::Bus_SPI _bus_instance;

public:
    LGFX_P4() {
        auto cfg = _bus_instance.config();
        cfg.spi_host = SPI3_HOST;
        cfg.spi_mode = 0;
        cfg.freq_write = 20000000; // Original speed
        cfg.pin_sclk = qpcr::pins::kDisplaySck;
        cfg.pin_mosi = qpcr::pins::kDisplayMosi;
        cfg.pin_dc = qpcr::pins::kDisplayDc;
        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);

        auto p_cfg = _panel_instance.config();
        p_cfg.pin_cs = qpcr::pins::kDisplayCs;
        p_cfg.pin_rst = qpcr::pins::kDisplayRst;
        p_cfg.panel_width = 320;
        p_cfg.panel_height = 480;
        _panel_instance.config(p_cfg);
        setPanel(&_panel_instance);
    }
};

LGFX_P4 lcd;

// Firmware version in the bottom-right corner of every screen (issue #28). Uses the
// display's reported size, so it stays in the corner after setRotation().
void drawVersionBadge() {
    char label[32];
    qpcr::ui::versionBadgeLabel(label, sizeof label, qpcr::firmwareVersion());
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    const qpcr::ui::TextMetrics metrics{static_cast<int>(lcd.textWidth(label)),
                                        static_cast<int>(lcd.fontHeight())};
    const qpcr::ui::Point origin = qpcr::ui::versionBadgeOrigin(lcd.width(), lcd.height(), metrics);
    lcd.setCursor(origin.x, origin.y);
    lcd.print(label);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.print("Portable qPCR firmware ");
    Serial.println(qpcr::firmwareVersion());

    if (lcd.init()) {
        lcd.setRotation(1);
        lcd.fillScreen(TFT_BLACK);

        // The exact text from last night
        lcd.setTextColor(TFT_WHITE);
        lcd.setTextSize(3);
        lcd.setCursor(20, 20);
        lcd.print("SYSTEM VERIFIED");

        lcd.setTextColor(TFT_GREEN);
        lcd.setCursor(20, 80);
        lcd.print("Brain: ONLINE");

        lcd.setTextColor(TFT_BLUE);
        lcd.setCursor(20, 140);
        lcd.print("Screen: ILI9488");

        drawVersionBadge();
    }
}

void loop() {
    delay(1000);
}