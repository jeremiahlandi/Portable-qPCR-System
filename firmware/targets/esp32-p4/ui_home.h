#pragma once

#include <lvgl.h>

// Home screen (issue #11): status bar, a 2x2 grid of launcher tiles (placeholders until #12,
// Settings is live as of #37), and the version badge bottom-right (#28).

namespace qpcr::target {

/// Builder for the navigation stack.
lv_obj_t* uiHomeBuild();

/// Shows the home screen and resets navigation. Call once, with the LVGL lock held.
void uiHomeCreate();

} // namespace qpcr::target
