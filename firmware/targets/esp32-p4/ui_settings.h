#pragma once

#include <lvgl.h>

// Settings screens (issue #37): landing list, Network (Wi-Fi + Bluetooth), Time, Device info.
// Each builder returns a screen for the navigation stack. Poll functions refresh live values
// on the active screen; call them from loop() with the LVGL lock held.

namespace qpcr::target {

lv_obj_t* uiSettingsBuild();
lv_obj_t* uiNetworkBuild();
lv_obj_t* uiTimeBuild();
lv_obj_t* uiDeviceBuild();

void uiSettingsPoll();

} // namespace qpcr::target
