#include "ui_home.h"

#include <array>
#include <cstddef>
#include <lvgl.h>

#include "core/Version.h"
#include "core/ui/Launcher.h"
#include "core/ui/StatusBar.h"
#include "core/ui/VersionBadge.h"

#include "display.h"

namespace qpcr::target {

namespace {

constexpr int kBarHeight = 28;
constexpr int kPad = 8;
constexpr lv_opa_t kDimmed = LV_OPA_40;

constexpr std::uint32_t kBackground = 0x101418;
constexpr std::uint32_t kBarBackground = 0x1c2229;
constexpr std::uint32_t kText = 0xf2f4f6;
constexpr std::array<std::uint32_t, 4> kTileColors{0xd63a3a, 0x2f6fde, 0x2a9d5c, 0x6b7280};
constexpr std::array<const char*, 4> kTileSymbols{LV_SYMBOL_PLAY, LV_SYMBOL_LIST, LV_SYMBOL_FILE,
                                                  LV_SYMBOL_SETTINGS};

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
    lv_obj_set_style_text_color(label, lv_color_hex(kText), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    return label;
}

void buildStatusBar(lv_obj_t* screen) {
    lv_obj_t* bar = lv_obj_create(screen);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, kScreenWidth, kBarHeight);
    lv_obj_set_pos(bar, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(kBarBackground), 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_left(bar, kPad, 0);
    lv_obj_set_style_pad_right(bar, kPad, 0);

    clockLabel = makeIcon(bar, "00:00");
    lv_obj_align(clockLabel, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t* right = lv_obj_create(bar);
    lv_obj_remove_style_all(right);
    lv_obj_set_size(right, LV_SIZE_CONTENT, kBarHeight);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(right, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(right, kPad, 0);
    lv_obj_align(right, LV_ALIGN_RIGHT_MID, 0, 0);

    wifiLabel = makeIcon(right, LV_SYMBOL_WIFI);
    bluetoothLabel = makeIcon(right, LV_SYMBOL_BLUETOOTH);
    batteryLabel = makeIcon(right, LV_SYMBOL_CHARGE);
}

void onTileClicked(lv_event_t* e) {
    const auto index = reinterpret_cast<std::uintptr_t>(lv_event_get_user_data(e));
    // Placeholder until #12: navigation is routed on the tile id.
    (void)index;
}

void buildTiles(lv_obj_t* screen) {
    constexpr int kTile = 104;
    constexpr int kGap = 24;
    constexpr int kGridWidth = 2 * kTile + kGap;
    constexpr int kGridHeight = 2 * kTile + kGap;

    lv_obj_t* grid = lv_obj_create(screen);
    lv_obj_remove_style_all(grid);
    lv_obj_set_size(grid, kGridWidth, kGridHeight);
    lv_obj_align(grid, LV_ALIGN_CENTER, 0, kBarHeight / 2);
    static const std::array<int32_t, 3> cols{kTile, kTile, LV_GRID_TEMPLATE_LAST};
    static const std::array<int32_t, 3> rows{kTile, kTile, LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(grid, cols.data(), rows.data());
    lv_obj_set_style_pad_column(grid, kGap, 0);
    lv_obj_set_style_pad_row(grid, kGap, 0);

    for (std::size_t i = 0; i < qpcr::ui::kLauncherTiles.size(); ++i) {
        const auto& tile = qpcr::ui::kLauncherTiles.at(i);
        lv_obj_t* button = lv_button_create(grid);
        lv_obj_set_size(button, kTile, kTile);
        lv_obj_set_grid_cell(button, LV_GRID_ALIGN_STRETCH, static_cast<int32_t>(i % 2), 1,
                             LV_GRID_ALIGN_STRETCH, static_cast<int32_t>(i / 2), 1);
        lv_obj_set_style_bg_color(button, lv_color_hex(kTileColors.at(i)), 0);
        lv_obj_set_style_radius(button, 20, 0);
        lv_obj_set_style_shadow_width(button, 0, 0);
        lv_obj_add_event_cb(button, onTileClicked, LV_EVENT_CLICKED,
                            reinterpret_cast<void*>(static_cast<std::uintptr_t>(i)));

        lv_obj_t* icon = lv_label_create(button);
        lv_label_set_text(icon, kTileSymbols.at(i));
        lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(icon, lv_color_hex(kText), 0);
        lv_obj_align(icon, LV_ALIGN_CENTER, 0, -12);

        lv_obj_t* label = lv_label_create(button);
        lv_label_set_text(label, tile.label);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(label, lv_color_hex(kText), 0);
        lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -6);
    }
}

void buildVersionBadge(lv_obj_t* screen) {
    char text[32];
    qpcr::ui::versionBadgeLabel(text, sizeof text, qpcr::firmwareVersion());
    lv_obj_t* label = lv_label_create(screen);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(kText), 0);
    lv_obj_set_style_text_opa(label, LV_OPA_70, 0);
    lv_obj_align(label, LV_ALIGN_BOTTOM_RIGHT, -qpcr::ui::kVersionBadgeMargin,
                 -qpcr::ui::kVersionBadgeMargin);
}

} // namespace

void uiHomeCreate() {
    lv_obj_t* screen = lv_obj_create(nullptr);
    lv_obj_remove_style_all(screen);
    lv_obj_set_size(screen, kScreenWidth, kScreenHeight);
    lv_obj_set_style_bg_color(screen, lv_color_hex(kBackground), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    buildStatusBar(screen);
    buildTiles(screen);
    buildVersionBadge(screen);
    lv_screen_load(screen);
}

void uiHomeSetStatus(const qpcr::ui::StatusBarState& state) {
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
