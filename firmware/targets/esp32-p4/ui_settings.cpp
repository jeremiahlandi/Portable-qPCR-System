#include "ui_settings.h"

#include <array>
#include <cstdio>
#include <cstring>

#include "core/Version.h"
#include "core/net/WifiSettings.h"
#include "core/time/TimeZones.h"

#include "display.h"
#include "net_wifi.h"
#include "time_sync.h"
#include "ui_nav.h"
#include "ui_theme.h"

namespace qpcr::target {

namespace {

using qpcr::net::ConnectState;

// ---- shared widgets -------------------------------------------------------------------

lv_obj_t* makeLabel(lv_obj_t* parent, const char* text, const lv_font_t* font,
                    std::uint32_t colour) {
    lv_obj_t* label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(colour), 0);
    return label;
}

lv_obj_t* makeButton(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* user) {
    lv_obj_t* button = lv_button_create(parent);
    lv_obj_set_style_bg_color(button, lv_color_hex(theme::kAccent), 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_set_style_radius(button, 8, 0);
    lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED, user);
    lv_obj_t* label = makeLabel(button, text, &lv_font_montserrat_16, theme::kText);
    lv_obj_center(label);
    return button;
}

// A row with a label on the left and a switch on the right.
lv_obj_t* makeSwitchRow(lv_obj_t* parent, const char* text, bool on, lv_event_cb_t cb) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, LV_PCT(100), 36);
    makeLabel(row, text, &lv_font_montserrat_16, theme::kText);
    lv_obj_t* sw = lv_switch_create(row);
    lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
    if (on) {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(sw, cb, LV_EVENT_VALUE_CHANGED, nullptr);
    return sw;
}

// ---- Settings landing -------------------------------------------------------------------

void onNetwork(lv_event_t* /*e*/) {
    uiNavPush(uiNetworkBuild);
}
void onTime(lv_event_t* /*e*/) {
    uiNavPush(uiTimeBuild);
}
void onDevice(lv_event_t* /*e*/) {
    uiNavPush(uiDeviceBuild);
}

lv_obj_t* addListItem(lv_obj_t* list, const char* icon, const char* text, lv_event_cb_t cb) {
    lv_obj_t* item = lv_list_add_button(list, icon, text);
    lv_obj_set_style_bg_color(item, lv_color_hex(theme::kPanel), 0);
    lv_obj_set_style_text_color(item, lv_color_hex(theme::kText), 0);
    lv_obj_set_style_text_font(item, &lv_font_montserrat_16, 0);
    lv_obj_add_event_cb(item, cb, LV_EVENT_CLICKED, nullptr);
    return item;
}

// ---- Network ----------------------------------------------------------------------------

lv_obj_t* netStatusLabel = nullptr;
lv_obj_t* netList = nullptr;
lv_obj_t* netPasswordPanel = nullptr;
lv_obj_t* netPasswordArea = nullptr;
lv_obj_t* netKeyboard = nullptr;
lv_obj_t* netScanButton = nullptr;
std::array<NetworkInfo, 16> netResults{};
std::size_t netResultCount = 0;
char netChosenSsid[qpcr::net::kSsidMax + 1] = {};
bool netListStale = true;
ConnectState netShownState = ConnectState::Idle;

void onNetworkDeleted(lv_event_t* /*e*/) {
    netStatusLabel = nullptr;
    netList = nullptr;
    netPasswordPanel = nullptr;
    netPasswordArea = nullptr;
    netKeyboard = nullptr;
    netScanButton = nullptr;
}

void refreshNetStatus() {
    if (netStatusLabel == nullptr) {
        return;
    }
    const ConnectState s = netState();
    char text[96];
    if (s == ConnectState::Connected) {
        char ip[20];
        netIp(ip, sizeof ip);
        std::snprintf(text, sizeof text, "Connected to %s  (%s, %d dBm)", netSsid(), ip, netRssi());
    } else if (netSsid()[0] != '\0' && (s == ConnectState::Idle || qpcr::net::isFailure(s))) {
        std::snprintf(text, sizeof text, "%s: %s", netSsid(), qpcr::net::stateText(s));
    } else {
        std::snprintf(text, sizeof text, "%s", qpcr::net::stateText(s));
    }
    lv_label_set_text(netStatusLabel, text);
    const bool good = s == ConnectState::Connected;
    const bool bad = qpcr::net::isFailure(s);
    lv_obj_set_style_text_color(
        netStatusLabel, lv_color_hex(good ? theme::kGood : (bad ? theme::kBad : theme::kMuted)), 0);
}

void hidePasswordPanel() {
    if (netPasswordPanel != nullptr) {
        lv_obj_add_flag(netPasswordPanel, LV_OBJ_FLAG_HIDDEN);
    }
}

void onPasswordReady(lv_event_t* /*e*/) {
    if (netPasswordArea == nullptr) {
        return;
    }
    const char* pass = lv_textarea_get_text(netPasswordArea);
    if (!netConnect(netChosenSsid, pass)) {
        if (netStatusLabel != nullptr) {
            lv_label_set_text(netStatusLabel, "Password must be 8 to 63 characters");
            lv_obj_set_style_text_color(netStatusLabel, lv_color_hex(theme::kBad), 0);
        }
        return;
    }
    hidePasswordPanel();
    refreshNetStatus();
}

void onPasswordCancel(lv_event_t* /*e*/) {
    hidePasswordPanel();
}

void onShowPassword(lv_event_t* e) {
    if (netPasswordArea == nullptr) {
        return;
    }
    auto* sw = static_cast<lv_obj_t*>(lv_event_get_target(e));
    lv_textarea_set_password_mode(netPasswordArea, !lv_obj_has_state(sw, LV_STATE_CHECKED));
}

void onNetworkChosen(lv_event_t* e) {
    const auto index = reinterpret_cast<std::uintptr_t>(lv_event_get_user_data(e));
    if (index >= netResultCount) {
        return;
    }
    const NetworkInfo& n = netResults.at(index);
    std::strncpy(netChosenSsid, n.ssid, sizeof netChosenSsid - 1);
    if (!n.secured) {
        netConnect(netChosenSsid, "");
        refreshNetStatus();
        return;
    }
    if (netPasswordPanel != nullptr) {
        lv_textarea_set_text(netPasswordArea, "");
        lv_obj_remove_flag(netPasswordPanel, LV_OBJ_FLAG_HIDDEN);
        lv_keyboard_set_mode(netKeyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    }
}

void fillNetList() {
    if (netList == nullptr) {
        return;
    }
    lv_obj_clean(netList);
    netResultCount = 0;
    const std::size_t count = netScanCount();
    for (std::size_t i = 0; i < count && netResultCount < netResults.size(); ++i) {
        const NetworkInfo info = netScanResult(i);
        if (info.ssid[0] == '\0') {
            continue; // hidden network
        }
        netResults.at(netResultCount) = info;
        char text[64];
        std::snprintf(text, sizeof text, "%s   %d dBm", info.ssid, info.rssi);
        lv_obj_t* item =
            lv_list_add_button(netList, info.secured ? LV_SYMBOL_CLOSE : nullptr, text);
        lv_obj_set_style_bg_color(item, lv_color_hex(theme::kPanel), 0);
        lv_obj_set_style_text_color(item, lv_color_hex(theme::kText), 0);
        lv_obj_add_event_cb(item, onNetworkChosen, LV_EVENT_CLICKED,
                            reinterpret_cast<void*>(static_cast<std::uintptr_t>(netResultCount)));
        ++netResultCount;
    }
    if (netResultCount == 0) {
        lv_obj_t* none = lv_list_add_text(netList, "No networks found");
        lv_obj_set_style_text_color(none, lv_color_hex(theme::kMuted), 0);
    }
}

void onScan(lv_event_t* /*e*/) {
    netScanStart();
    netListStale = true;
    if (netList != nullptr) {
        lv_obj_clean(netList);
        lv_obj_t* scanning = lv_list_add_text(netList, "Scanning...");
        lv_obj_set_style_text_color(scanning, lv_color_hex(theme::kMuted), 0);
    }
    refreshNetStatus();
}

void onForget(lv_event_t* /*e*/) {
    netForget();
    refreshNetStatus();
}

void onWifiSwitch(lv_event_t* e) {
    auto* sw = static_cast<lv_obj_t*>(lv_event_get_target(e));
    netSetWifiEnabled(lv_obj_has_state(sw, LV_STATE_CHECKED));
    refreshNetStatus();
}

void onBluetoothSwitch(lv_event_t* e) {
    auto* sw = static_cast<lv_obj_t*>(lv_event_get_target(e));
    netSetBluetoothEnabled(lv_obj_has_state(sw, LV_STATE_CHECKED));
}

// ---- Time -------------------------------------------------------------------------------

lv_obj_t* timeClockLabel = nullptr;
lv_obj_t* timeDateLabel = nullptr;
lv_obj_t* timeSyncLabel = nullptr;

void onTimeDeleted(lv_event_t* /*e*/) {
    timeClockLabel = nullptr;
    timeDateLabel = nullptr;
    timeSyncLabel = nullptr;
}

void refreshTime() {
    if (timeClockLabel == nullptr) {
        return;
    }
    char text[24];
    timeClockText(text, sizeof text);
    lv_label_set_text(timeClockLabel, text);
    timeDateText(text, sizeof text);
    lv_label_set_text(timeDateLabel, text);
    if (timeSynced()) {
        lv_label_set_text(timeSyncLabel, "Synced over the internet");
        lv_obj_set_style_text_color(timeSyncLabel, lv_color_hex(theme::kGood), 0);
    } else if (netState() == ConnectState::Connected) {
        lv_label_set_text(timeSyncLabel, "Waiting for the time server...");
        lv_obj_set_style_text_color(timeSyncLabel, lv_color_hex(theme::kMuted), 0);
    } else {
        lv_label_set_text(timeSyncLabel, "Join Wi-Fi to set the time");
        lv_obj_set_style_text_color(timeSyncLabel, lv_color_hex(theme::kMuted), 0);
    }
}

void onZoneChanged(lv_event_t* e) {
    auto* dropdown = static_cast<lv_obj_t*>(lv_event_get_target(e));
    timeSetZone(lv_dropdown_get_selected(dropdown));
    refreshTime();
}

void onSyncNow(lv_event_t* /*e*/) {
    timeResync();
    refreshTime();
}

} // namespace

lv_obj_t* uiSettingsBuild() {
    lv_obj_t* content = uiNavMakeScreen("Settings");
    lv_obj_t* list = lv_list_create(content);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(list, lv_color_hex(theme::kBackground), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    addListItem(list, LV_SYMBOL_WIFI, "Network", onNetwork);
    addListItem(list, LV_SYMBOL_BELL, "Time", onTime);
    addListItem(list, LV_SYMBOL_SETTINGS, "Device info", onDevice);
    return lv_obj_get_screen(content);
}

lv_obj_t* uiNetworkBuild() {
    lv_obj_t* content = uiNavMakeScreen("Network");
    lv_obj_add_event_cb(content, onNetworkDeleted, LV_EVENT_DELETE, nullptr);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(content, 6, 0);

    makeSwitchRow(content, "Wi-Fi", netWifiEnabled(), onWifiSwitch);

    netStatusLabel = makeLabel(content, "", &lv_font_montserrat_14, theme::kMuted);
    lv_obj_set_width(netStatusLabel, LV_PCT(100));
    lv_label_set_long_mode(netStatusLabel, LV_LABEL_LONG_DOT);

    lv_obj_t* buttons = lv_obj_create(content);
    lv_obj_remove_style_all(buttons);
    lv_obj_set_size(buttons, LV_PCT(100), 36);
    lv_obj_set_flex_flow(buttons, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(buttons, theme::kPad, 0);
    netScanButton = makeButton(buttons, LV_SYMBOL_REFRESH " Scan", onScan, nullptr);
    makeButton(buttons, "Forget", onForget, nullptr);

    netList = lv_list_create(content);
    lv_obj_set_width(netList, LV_PCT(100));
    lv_obj_set_flex_grow(netList, 1);
    lv_obj_set_style_bg_color(netList, lv_color_hex(theme::kBackground), 0);
    lv_obj_set_style_border_width(netList, 0, 0);
    lv_obj_t* hint = lv_list_add_text(netList, "Tap Scan to list networks");
    lv_obj_set_style_text_color(hint, lv_color_hex(theme::kMuted), 0);

    makeSwitchRow(content,
                  netBluetoothSupported() ? "Bluetooth" : "Bluetooth (radio not in this build)",
                  netBluetoothEnabled(), onBluetoothSwitch);

    // Password entry: an overlay on the same screen with a text area and the keyboard.
    lv_obj_t* screen = lv_obj_get_screen(content);
    netPasswordPanel = lv_obj_create(screen);
    lv_obj_remove_style_all(netPasswordPanel);
    lv_obj_set_size(netPasswordPanel, kScreenWidth, kScreenHeight - theme::kBarHeight);
    lv_obj_set_pos(netPasswordPanel, 0, theme::kBarHeight);
    lv_obj_set_style_bg_color(netPasswordPanel, lv_color_hex(theme::kBackground), 0);
    lv_obj_set_style_bg_opa(netPasswordPanel, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(netPasswordPanel, theme::kPad, 0);
    lv_obj_add_flag(netPasswordPanel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t* prompt =
        makeLabel(netPasswordPanel, "Password", &lv_font_montserrat_16, theme::kText);
    lv_obj_align(prompt, LV_ALIGN_TOP_LEFT, 0, 4);

    lv_obj_t* show = lv_switch_create(netPasswordPanel);
    lv_obj_align(show, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_add_event_cb(show, onShowPassword, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_t* showLabel =
        makeLabel(netPasswordPanel, "Show", &lv_font_montserrat_14, theme::kMuted);
    lv_obj_align_to(showLabel, show, LV_ALIGN_OUT_LEFT_MID, -6, 0);

    netPasswordArea = lv_textarea_create(netPasswordPanel);
    lv_obj_set_size(netPasswordArea, kScreenWidth - 2 * theme::kPad, 36);
    lv_obj_align(netPasswordArea, LV_ALIGN_TOP_MID, 0, 32);
    lv_textarea_set_one_line(netPasswordArea, true);
    lv_textarea_set_password_mode(netPasswordArea, true);
    lv_textarea_set_max_length(netPasswordArea, qpcr::net::kPassphraseMax);
    lv_textarea_set_placeholder_text(netPasswordArea, "8 to 63 characters");

    netKeyboard = lv_keyboard_create(netPasswordPanel);
    lv_obj_set_size(netKeyboard, kScreenWidth - 2 * theme::kPad,
                    kScreenHeight - theme::kBarHeight - 84);
    lv_obj_align(netKeyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(netKeyboard, netPasswordArea);
    lv_obj_add_event_cb(netKeyboard, onPasswordReady, LV_EVENT_READY, nullptr);
    lv_obj_add_event_cb(netKeyboard, onPasswordCancel, LV_EVENT_CANCEL, nullptr);

    netListStale = !netScanning() && netScanCount() > 0;
    if (netListStale) {
        fillNetList();
        netListStale = false;
    }
    netShownState = netState();
    refreshNetStatus();
    return screen;
}

lv_obj_t* uiTimeBuild() {
    lv_obj_t* content = uiNavMakeScreen("Time");
    lv_obj_add_event_cb(content, onTimeDeleted, LV_EVENT_DELETE, nullptr);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(content, 10, 0);

    timeClockLabel = makeLabel(content, "--:--:--", &lv_font_montserrat_28, theme::kText);
    timeDateLabel = makeLabel(content, "not set", &lv_font_montserrat_16, theme::kMuted);
    timeSyncLabel = makeLabel(content, "", &lv_font_montserrat_14, theme::kMuted);

    lv_obj_t* row = lv_obj_create(content);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, LV_PCT(100), 40);
    makeLabel(row, "Time zone", &lv_font_montserrat_16, theme::kText);

    static char options[256];
    options[0] = '\0';
    for (std::size_t i = 0; i < qpcr::time::kTimeZones.size(); ++i) {
        std::strncat(options, qpcr::time::kTimeZones.at(i).label,
                     sizeof options - std::strlen(options) - 1);
        if (i + 1 < qpcr::time::kTimeZones.size()) {
            std::strncat(options, "\n", sizeof options - std::strlen(options) - 1);
        }
    }
    lv_obj_t* dropdown = lv_dropdown_create(row);
    lv_dropdown_set_options_static(dropdown, options);
    lv_dropdown_set_selected(dropdown, static_cast<std::uint32_t>(timeZoneSelected()));
    lv_obj_set_width(dropdown, 200);
    lv_obj_align(dropdown, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(dropdown, onZoneChanged, LV_EVENT_VALUE_CHANGED, nullptr);

    makeButton(content, LV_SYMBOL_REFRESH " Sync now", onSyncNow, nullptr);
    refreshTime();
    return lv_obj_get_screen(content);
}

lv_obj_t* uiDeviceBuild() {
    lv_obj_t* content = uiNavMakeScreen("Device info");
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(content, 6, 0);
    char line[96];
    std::snprintf(line, sizeof line, "Firmware  v%s", qpcr::firmwareVersion());
    makeLabel(content, line, &lv_font_montserrat_16, theme::kText);
    makeLabel(content, "Controller  ESP32-P4 + ESP32-C6 radio", &lv_font_montserrat_16,
              theme::kText);
    char ip[20];
    netIp(ip, sizeof ip);
    std::snprintf(line, sizeof line, "Wi-Fi  %s  %s", netSsid()[0] != '\0' ? netSsid() : "-", ip);
    makeLabel(content, line, &lv_font_montserrat_16, theme::kText);
    std::snprintf(line, sizeof line, "Time zone  %s",
                  qpcr::time::kTimeZones.at(timeZoneSelected()).label);
    makeLabel(content, line, &lv_font_montserrat_16, theme::kText);
    return lv_obj_get_screen(content);
}

void uiSettingsPoll() {
    if (netList != nullptr) {
        if (netListStale && !netScanning()) {
            fillNetList();
            netListStale = false;
        }
        const ConnectState s = netState();
        if (s != netShownState) {
            netShownState = s;
            refreshNetStatus();
        }
    }
    refreshTime();
}

} // namespace qpcr::target
