#pragma once

#include "core/ui/StatusBar.h"

// Home screen (issue #11): status bar along the top (clock, Wi-Fi, Bluetooth, battery), a
// 2x2 grid of launcher tiles (placeholders until #12), and the version badge bottom-right (#28).
// Layout follows the board's factory launcher so the operator's first impression matches.

namespace qpcr::target {

/// Builds the screen and loads it. Call once, with the LVGL lock held.
void uiHomeCreate();

/// Pushes new status bar values. Call with the LVGL lock held.
void uiHomeSetStatus(const qpcr::ui::StatusBarState& state);

} // namespace qpcr::target
