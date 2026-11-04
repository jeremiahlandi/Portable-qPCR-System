#include "touch_ft6336.h"

#include <Arduino.h>
#include <Wire.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <lvgl.h>

#include "display.h"
#include "pins.h"

namespace qpcr::target {

namespace {

constexpr std::uint8_t kAddress = 0x38;
constexpr std::uint8_t kRegTouchCount = 0x02; // low nibble: number of points (0..2)
constexpr std::uint8_t kRegPoint1 = 0x03;     // XH, XL, YH, YL
constexpr std::uint32_t kI2cHz = 400000;
constexpr int kPanelWidth = 320; // native (portrait) panel size the controller reports in
constexpr int kPanelHeight = 480;

bool present = false;

struct Point {
    int x;
    int y;
    bool pressed;
};

// Reads the first touch point in native panel coordinates.
Point readNative() {
    Point p{0, 0, false};
    Wire.beginTransmission(kAddress);
    Wire.write(kRegTouchCount);
    if (Wire.endTransmission(false) != 0) {
        return p;
    }
    constexpr std::uint8_t kLen = 5; // count + XH XL YH YL
    if (Wire.requestFrom(kAddress, kLen) != kLen) {
        return p;
    }
    std::array<std::uint8_t, kLen> raw{};
    std::generate(raw.begin(), raw.end(), [] { return static_cast<std::uint8_t>(Wire.read()); });
    const std::uint8_t count = raw[0] & 0x0F;
    if (count == 0 || count > 2) {
        return p;
    }
    p.x = ((raw[1] & 0x0F) << 8) | raw[2];
    p.y = ((raw[3] & 0x0F) << 8) | raw[4];
    p.pressed = true;
    return p;
}

// Native portrait (320x480) to the landscape screen (LovyanGFX rotation 1). If touches land
// mirrored on the bench, swap the two expressions below; this is the only place to change.
Point toScreen(Point native) {
    Point s = native;
    s.x = native.y;
    s.y = (kPanelWidth - 1) - native.x;
    s.x = std::max(0, std::min(s.x, kScreenWidth - 1));
    s.y = std::max(0, std::min(s.y, kScreenHeight - 1));
    return s;
}

void readCb(lv_indev_t* /*indev*/, lv_indev_data_t* data) {
    const Point p = toScreen(readNative());
    data->state = p.pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    if (p.pressed) {
        data->point.x = p.x;
        data->point.y = p.y;
    }
}

} // namespace

bool touchInit() {
    // TP_RST (GPIO29) has a 10 k pull-down on the board, so the FT6336 is held in reset until we
    // release it. Pulse low, then high, then give the controller its start-up time (datasheet:
    // >= 300 ms from reset to first valid I2C access) before probing.
    pinMode(qpcr::pins::kTouchRst, OUTPUT);
    digitalWrite(qpcr::pins::kTouchRst, LOW);
    delay(10);
    digitalWrite(qpcr::pins::kTouchRst, HIGH);
    delay(300);
    Wire.begin(qpcr::pins::kI2cSda, qpcr::pins::kI2cScl, kI2cHz);
    Wire.beginTransmission(kAddress);
    present = Wire.endTransmission() == 0;
    return present;
}

void touchAttachToLvgl() {
    if (!present) {
        return;
    }
    lv_indev_t* indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, readCb);
}

} // namespace qpcr::target
