#pragma once

#include <lvgl.h>

#include "core/ui/StatusBar.h"

// Status bar along the top of every screen (issue #11): clock left, Wi-Fi, Bluetooth and
// battery glyphs right. One instance lives on the active screen; update() finds it.

namespace qpcr::target {

/// Creates the bar on `screen` at the top. Call with the LVGL lock held.
lv_obj_t* uiStatusBarCreate(lv_obj_t* screen);

/// Pushes new values to the bar on the active screen, if any. Call with the LVGL lock held.
void uiStatusBarUpdate(const qpcr::ui::StatusBarState& state);

} // namespace qpcr::target
