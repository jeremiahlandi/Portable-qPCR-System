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
#include "net_wifi.h"
#include "peltier.h"
#include "time_sync.h"
#include "touch_ft6336.h"
#include "ui_home.h"
#include "ui_settings.h"
#include "ui_statusbar.h"

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
    // Peltier H-bridge pins go to their safe state before anything else runs. The drive stays
    // disabled: no control loop commands it yet (#4, #5), so the bridge never drives at boot.
    const bool peltierOk = qpcr::target::peltierInit();

    Serial.begin(115200);
    delay(1000);
    Serial.print("Portable qPCR firmware ");
    Serial.println(qpcr::firmwareBuild());
    Serial.print("Peltier H-bridge: ");
    Serial.println(peltierOk ? "safe state, drive disabled" : "LEDC attach FAILED, drive inert");

    lvglMutex = xSemaphoreCreateMutex();
    lv_init();
    lv_tick_set_cb(tickMs);

    qpcr::target::netInit();
    qpcr::target::timeInit();

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
    xTaskCreatePinnedToCore(lvglTask, "lvgl", 12288, nullptr, 2, nullptr, 1);
}

void loop() {
    qpcr::target::netPoll();
    qpcr::target::timePoll();

    static std::uint32_t lastUiMs = 0;
    const std::uint32_t nowMs = millis();
    if (uiEnabled && nowMs - lastUiMs >= 500U) {
        lastUiMs = nowMs;
        qpcr::ui::StatusBarState state;
        state.secondsOfDay = qpcr::target::timeSecondsOfDay();
        state.wifiConnected = qpcr::target::netState() == qpcr::net::ConnectState::Connected;
        state.bluetoothConnected =
            qpcr::target::netBluetoothEnabled() && qpcr::target::netBluetoothSupported();
        state.batteryPercent = -1; // no gauge yet; the unit runs from the wall supply
        LvglLock lock;
        qpcr::target::uiStatusBarUpdate(state);
        qpcr::target::uiSettingsPoll();
    }
    delay(20);
}
