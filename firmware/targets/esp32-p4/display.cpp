#include "display.h"

#include <Arduino.h>
#include <algorithm>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <esp_heap_caps.h>
#include <lvgl.h>

#include "pins.h"

namespace qpcr::target {

namespace {

// ST7796 320x480 over SPI, routed by the board. IPS panel (inverted colours) as in Waveshare's
// Arduino_GFX board library. Their "rotation 4" maps to LovyanGFX offset_rotation 0: with 4 the
// image came out X-mirrored (verified 2026-11-01, #31).
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

// Two partial render buffers of 60 lines each (57.6 KB, RGB565) in internal DMA-capable RAM.
// Partial mode pushes only the areas LVGL redrew, which keeps a mostly static screen cheap on
// the 40 MHz SPI link. Full-frame buffers in PSRAM are the fallback if tearing shows up.
constexpr int kBufferLines = 60;
constexpr std::size_t kBufferBytes =
    static_cast<std::size_t>(kScreenWidth) * kBufferLines * sizeof(std::uint16_t);

void flushCb(lv_display_t* disp, const lv_area_t* area, std::uint8_t* pxMap) {
    const std::int32_t w = lv_area_get_width(area);
    const std::int32_t h = lv_area_get_height(area);
    lcd.startWrite();
    lcd.setAddrWindow(area->x1, area->y1, w, h);
    // LVGL renders native-order RGB565; LovyanGFX swaps to the panel's byte order itself.
    // cppcheck-suppress cstyleCast ; LVGL hands over a byte pointer to RGB565 pixels
    lcd.writePixels(reinterpret_cast<const lgfx::rgb565_t*>(pxMap),
                    static_cast<std::uint32_t>(w) * static_cast<std::uint32_t>(h));
    lcd.endWrite();
    lv_display_flush_ready(disp);
}

} // namespace

void displaySetBacklight(std::uint8_t percent) {
    static bool attached = false;
    if (!attached) {
        attached = ledcAttach(qpcr::pins::kDisplayBacklight, 5000, 10);
    }
    if (attached) {
        ledcWrite(qpcr::pins::kDisplayBacklight,
                  (1023U * std::min<std::uint8_t>(percent, 100)) / 100U);
    }
}

bool displayInit() {
    displaySetBacklight(0);
    if (!lcd.init()) {
        return false;
    }
    lcd.setRotation(1); // landscape, 480x320
    lcd.fillScreen(TFT_BLACK);

    auto* buf1 = heap_caps_malloc(kBufferBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    auto* buf2 = heap_caps_malloc(kBufferBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buf1 == nullptr || buf2 == nullptr) {
        Serial.println("Display: LVGL buffer allocation failed");
        return false;
    }

    lv_display_t* disp = lv_display_create(kScreenWidth, kScreenHeight);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(disp, buf1, buf2, kBufferBytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, flushCb);
    return true;
}

} // namespace qpcr::target
