#pragma once

#include <lvgl.h>

// Screen navigation (issue #11): a small stack of screen builders. Screens are rebuilt when
// shown and deleted when left, so each one holds no state of its own; state lives in the
// modules the screens read (net_wifi, time_sync, core models).

namespace qpcr::target {

/// Builds a screen and returns it (not yet loaded).
using ScreenBuilder = lv_obj_t* (*)();

/// Creates a screen with the status bar and a header (title, back button when the stack is
/// deeper than the home screen). Returns the content container below the header, sized to
/// the rest of the display. Builders put their widgets in it.
lv_obj_t* uiNavMakeScreen(const char* title);

/// Shows the home screen and resets the stack. Call with the LVGL lock held.
void uiNavHome(ScreenBuilder home);

/// Pushes a screen onto the stack and shows it. Call with the LVGL lock held.
void uiNavPush(ScreenBuilder builder);

/// Returns to the previous screen. Call with the LVGL lock held.
void uiNavBack();

} // namespace qpcr::target
