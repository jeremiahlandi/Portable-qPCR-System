#include "ui_nav.h"

#include <array>
#include <cstddef>

#include "display.h"
#include "ui_statusbar.h"
#include "ui_theme.h"

namespace qpcr::target {

namespace {

constexpr std::size_t kMaxDepth = 6;
constexpr std::uint32_t kAnimMs = 150;

std::array<ScreenBuilder, kMaxDepth> stack{};
std::size_t depth = 0;

void onBackClicked(lv_event_t* /*e*/) {
    uiNavBack();
}

void show(lv_obj_t* screen, lv_screen_load_anim_t anim) {
    // auto_del: the previous screen is deleted once the animation ends.
    lv_screen_load_anim(screen, anim, kAnimMs, 0, true);
}

} // namespace

lv_obj_t* uiNavMakeScreen(const char* title) {
    lv_obj_t* screen = lv_obj_create(nullptr);
    lv_obj_remove_style_all(screen);
    lv_obj_set_size(screen, kScreenWidth, kScreenHeight);
    lv_obj_set_style_bg_color(screen, lv_color_hex(theme::kBackground), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    uiStatusBarCreate(screen);

    int top = theme::kBarHeight;
    if (title != nullptr) {
        lv_obj_t* header = lv_obj_create(screen);
        lv_obj_remove_style_all(header);
        lv_obj_set_size(header, kScreenWidth, theme::kHeaderHeight);
        lv_obj_set_pos(header, 0, theme::kBarHeight);
        lv_obj_set_style_pad_left(header, theme::kPad, 0);

        if (depth > 1) {
            lv_obj_t* back = lv_button_create(header);
            lv_obj_set_size(back, 56, 30);
            lv_obj_align(back, LV_ALIGN_LEFT_MID, 0, 0);
            lv_obj_set_style_bg_color(back, lv_color_hex(theme::kPanel), 0);
            lv_obj_set_style_shadow_width(back, 0, 0);
            lv_obj_add_event_cb(back, onBackClicked, LV_EVENT_CLICKED, nullptr);
            lv_obj_t* arrow = lv_label_create(back);
            lv_label_set_text(arrow, LV_SYMBOL_LEFT);
            lv_obj_set_style_text_color(arrow, lv_color_hex(theme::kText), 0);
            lv_obj_center(arrow);
        }

        lv_obj_t* label = lv_label_create(header);
        lv_label_set_text(label, title);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(label, lv_color_hex(theme::kText), 0);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
        top += theme::kHeaderHeight;
    }

    lv_obj_t* content = lv_obj_create(screen);
    lv_obj_remove_style_all(content);
    lv_obj_set_size(content, kScreenWidth, kScreenHeight - top);
    lv_obj_set_pos(content, 0, top);
    lv_obj_set_style_pad_all(content, theme::kPad, 0);
    return content;
}

void uiNavHome(ScreenBuilder home) {
    depth = 1;
    stack[0] = home;
    show(home(), LV_SCR_LOAD_ANIM_FADE_IN);
}

void uiNavPush(ScreenBuilder builder) {
    if (depth >= kMaxDepth || builder == nullptr) {
        return;
    }
    stack.at(depth) = builder;
    ++depth;
    show(builder(), LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void uiNavBack() {
    if (depth <= 1) {
        return;
    }
    --depth;
    show(stack.at(depth - 1)(), LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

} // namespace qpcr::target
