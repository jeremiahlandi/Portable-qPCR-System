#include "ui_home.h"

#include <array>
#include <cstddef>
#include <cstring>

#include "core/Version.h"
#include "core/ui/Launcher.h"
#include "core/ui/VersionBadge.h"

#include "display.h"
#include "ui_nav.h"
#include "ui_settings.h"
#include "ui_theme.h"

namespace qpcr::target {

namespace {

constexpr std::array<std::uint32_t, 4> kTileColors{theme::kBad, theme::kAccent, theme::kGood,
                                                   0x6b7280};
constexpr std::array<const char*, 4> kTileSymbols{LV_SYMBOL_PLAY, LV_SYMBOL_LIST, LV_SYMBOL_FILE,
                                                  LV_SYMBOL_SETTINGS};

void onTileClicked(lv_event_t* e) {
    const auto index = reinterpret_cast<std::uintptr_t>(lv_event_get_user_data(e));
    if (index >= qpcr::ui::kLauncherTiles.size()) {
        return;
    }
    if (std::strcmp(qpcr::ui::kLauncherTiles.at(index).id, "settings") == 0) {
        uiNavPush(uiSettingsBuild);
    }
    // Run, Protocols and Results route in #12.
}

void buildTiles(lv_obj_t* content) {
    constexpr int kTile = 104;
    constexpr int kGap = 24;
    constexpr int kGridWidth = 2 * kTile + kGap;
    constexpr int kGridHeight = 2 * kTile + kGap;

    lv_obj_t* grid = lv_obj_create(content);
    lv_obj_remove_style_all(grid);
    lv_obj_set_size(grid, kGridWidth, kGridHeight);
    lv_obj_align(grid, LV_ALIGN_CENTER, 0, 0);
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
        lv_obj_set_style_text_color(icon, lv_color_hex(theme::kText), 0);
        lv_obj_align(icon, LV_ALIGN_CENTER, 0, -12);

        lv_obj_t* label = lv_label_create(button);
        lv_label_set_text(label, tile.label);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(label, lv_color_hex(theme::kText), 0);
        lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -6);
    }
}

void buildVersionBadge(lv_obj_t* screen) {
    char text[32];
    qpcr::ui::versionBadgeLabel(text, sizeof text, qpcr::firmwareBuild());
    lv_obj_t* label = lv_label_create(screen);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(theme::kText), 0);
    lv_obj_set_style_text_opa(label, LV_OPA_70, 0);
    lv_obj_align(label, LV_ALIGN_BOTTOM_RIGHT, -qpcr::ui::kVersionBadgeMargin,
                 -qpcr::ui::kVersionBadgeMargin);
}

} // namespace

lv_obj_t* uiHomeBuild() {
    lv_obj_t* content = uiNavMakeScreen(nullptr);
    lv_obj_t* screen = lv_obj_get_screen(content);
    buildTiles(content);
    buildVersionBadge(screen);
    return screen;
}

void uiHomeCreate() {
    uiNavHome(uiHomeBuild);
}

} // namespace qpcr::target
