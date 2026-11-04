#include <Arduino.h>
#include <algorithm>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <lvgl.h>

#include "core/Version.h"
#include "core/ui/StatusBar.h"

#include "display.h"
#include "touch_ft6336.h"
#include "ui_home.h"

namespace {

// LVGL is not thread-safe: every call goes through this lock. The UI task on the second core
// runs the timer handler; loop() on core 0 only pushes state (issue #11, acceptance criterion 3).
SemaphoreHandle_t lvglMutex = nullptr;
bool uiEnabled = false;

class LvglLock {
public:
    LvglLock() { xSemaphoreTake(lvglMutex, portMAX_DELAY); }
    ~LvglLock() { xSemaphoreGive(lvglMutex); }
    LvglLock(const LvglLock&) = delete;
    LvglLock& operator=(const LvglLock&) = delete;
};

void lvglTask(void* /*unused*/) {
    for (;;) {
        std::uint32_t waitMs = 5;
        {
            LvglLock lock;
            waitMs = lv_timer_handler();
        }
        vTaskDelay(pdMS_TO_TICKS(std::max<std::uint32_t>(1, std::min<std::uint32_t>(waitMs, 20))));
    }
}

std::uint32_t tickMs() {
    return static_cast<std::uint32_t>(millis());
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.print("Portable qPCR firmware ");
    Serial.println(qpcr::firmwareVersion());

    lvglMutex = xSemaphoreCreateMutex();
    lv_init();
    lv_tick_set_cb(tickMs);

    const bool displayOk = qpcr::target::displayInit();
    Serial.print("Display init: ");
    Serial.println(displayOk ? "OK (ST7796, LVGL)" : "FAILED, running without UI");
    if (!displayOk) {
        return;
    }

    const bool touchOk = qpcr::target::touchInit();
    Serial.print("Touch init: ");
    Serial.println(touchOk ? "OK (FT6336)" : "not found, display only");
    qpcr::target::touchAttachToLvgl();

    {
        LvglLock lock;
        qpcr::target::uiHomeCreate();
    }
    qpcr::target::displaySetBacklight(100);
    uiEnabled = true;
    xTaskCreatePinnedToCore(lvglTask, "lvgl", 8192, nullptr, 2, nullptr, 1);
}

void loop() {
    // Status bar demo state: clock from uptime until an RTC and the radios exist (#14, #15).
    static std::uint32_t lastSecond = 0;
    const std::uint32_t now = millis() / 1000U;
    if (uiEnabled && now != lastSecond) {
        lastSecond = now;
        qpcr::ui::StatusBarState state;
        state.secondsOfDay = now;
        state.wifiConnected = false;
        state.bluetoothConnected = false;
        state.batteryPercent = -1;
        LvglLock lock;
        qpcr::target::uiHomeSetStatus(state);
    }
    delay(50);
}
