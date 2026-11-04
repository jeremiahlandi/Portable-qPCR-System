#include "ui_statusbar.h"

#include "display.h"
#include "ui_theme.h"

namespace qpcr::target {

namespace {

constexpr lv_opa_t kDimmed = LV_OPA_40;

// The bar on the screen being shown. Every screen builds its own bar (ui_nav), and LVGL
// deletes the previous screen only after the slide animation, by which time these pointers
// already belong to the new bar. onDeleted therefore checks the bar it is called for.
lv_obj_t* currentBar = nullptr;
lv_obj_t* clockLabel = nullptr;
lv_obj_t* wifiLabel = nullptr;
lv_obj_t* bluetoothLabel = nullptr;
lv_obj_t* batteryLabel = nullptr;

const char* batterySymbol(const qpcr::ui::StatusBarState& s) {
    if (s.charging) {
        return LV_SYMBOL_CHARGE;
    }
    switch (qpcr::ui::batteryLevel(s.batteryPercent)) {
    case qpcr::ui::BatteryLevel::Empty:
        return LV_SYMBOL_BATTERY_EMPTY;
    case qpcr::ui::BatteryLevel::Low:
        return LV_SYMBOL_BATTERY_1;
    case qpcr::ui::BatteryLevel::Half:
        return LV_SYMBOL_BATTERY_2;
    case qpcr::ui::BatteryLevel::High:
        return LV_SYMBOL_BATTERY_3;
    case qpcr::ui::BatteryLevel::Full:
        return LV_SYMBOL_BATTERY_FULL;
    case qpcr::ui::BatteryLevel::Unknown:
    default:
        return LV_SYMBOL_CHARGE; // no gauge yet: the unit runs from the wall supply
    }
}

lv_obj_t* makeIcon(lv_obj_t* parent, const char* symbol) {
    lv_obj_t* label = lv_label_create(parent);
    lv_label_set_text(label, symbol);
    lv_obj_set_style_text_color(label, lv_color_hex(theme::kText), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    return label;
}

void onDeleted(lv_event_t* e) {
    if (lv_event_get_target_obj(e) != currentBar) {
        return; // an older screen's bar going away: the labels belong to the new one
    }
    currentBar = nullptr;
    clockLabel = nullptr;
    wifiLabel = nullptr;
    bluetoothLabel = nullptr;
    batteryLabel = nullptr;
}

} // namespace

lv_obj_t* uiStatusBarCreate(lv_obj_t* screen) {
    lv_obj_t* bar = lv_obj_create(screen);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, kScreenWidth, theme::kBarHeight);
    lv_obj_set_pos(bar, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(theme::kBarBackground), 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_left(bar, theme::kPad, 0);
    lv_obj_set_style_pad_right(bar, theme::kPad, 0);
    lv_obj_add_event_cb(bar, onDeleted, LV_EVENT_DELETE, nullptr);
    currentBar = bar;

    clockLabel = makeIcon(bar, "--:--");
    lv_obj_align(clockLabel, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t* right = lv_obj_create(bar);
    lv_obj_remove_style_all(right);
    lv_obj_set_size(right, LV_SIZE_CONTENT, theme::kBarHeight);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(right, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(right, theme::kPad, 0);
    lv_obj_align(right, LV_ALIGN_RIGHT_MID, 0, 0);

    wifiLabel = makeIcon(right, LV_SYMBOL_WIFI);
    bluetoothLabel = makeIcon(right, LV_SYMBOL_BLUETOOTH);
    batteryLabel = makeIcon(right, LV_SYMBOL_CHARGE);
    return bar;
}

void uiStatusBarUpdate(const qpcr::ui::StatusBarState& state) {
    if (clockLabel == nullptr) {
        return;
    }
    char clock[8];
    qpcr::ui::formatClock(clock, sizeof clock, state.secondsOfDay);
    lv_label_set_text(clockLabel, clock);
    lv_obj_set_style_text_opa(
        wifiLabel, state.wifiConnected ? static_cast<lv_opa_t>(LV_OPA_COVER) : kDimmed, 0);
    lv_obj_set_style_text_opa(
        bluetoothLabel, state.bluetoothConnected ? static_cast<lv_opa_t>(LV_OPA_COVER) : kDimmed,
        0);
    lv_label_set_text(batteryLabel, batterySymbol(state));
}

} // namespace qpcr::target
