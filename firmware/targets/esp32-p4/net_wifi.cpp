#include "net_wifi.h"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <cstring>

namespace qpcr::target {

namespace {

using qpcr::net::ConnectEvent;
using qpcr::net::ConnectState;

constexpr const char* kNamespace = "net";
constexpr std::uint32_t kConnectTimeoutMs = 20000;
constexpr std::uint32_t kRetryMs = 10000;

Preferences prefs;
bool wifiOn = true;
bool btOn = false;
char storedSsid[qpcr::net::kSsidMax + 1] = {};
char storedPass[qpcr::net::kPassphraseMax + 1] = {};
ConnectState state = ConnectState::Idle;
bool scanning = false;
std::size_t scanCount = 0;
std::uint32_t connectStartMs = 0;
std::uint32_t lastDropMs = 0;

void apply(ConnectEvent e) {
    state = qpcr::net::nextState(state, e);
}

void startConnect() {
    if (!wifiOn || storedSsid[0] == '\0') {
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    WiFi.begin(storedSsid, storedPass[0] == '\0' ? nullptr : storedPass);
    connectStartMs = millis();
    state = ConnectState::Connecting;
}

void save() {
    prefs.putBool("wifi", wifiOn);
    prefs.putBool("bt", btOn);
    prefs.putString("ssid", storedSsid);
    prefs.putString("pass", storedPass);
}

} // namespace

void netInit() {
    prefs.begin(kNamespace, false);
    wifiOn = prefs.getBool("wifi", true);
    btOn = prefs.getBool("bt", false);
    prefs.getString("ssid", storedSsid, sizeof storedSsid);
    prefs.getString("pass", storedPass, sizeof storedPass);
    if (wifiOn) {
        WiFi.mode(WIFI_STA);
        startConnect();
    } else {
        WiFi.mode(WIFI_OFF);
    }
}

void netPoll() {
    if (scanning) {
        const int16_t r = WiFi.scanComplete();
        if (r >= 0) {
            scanCount = static_cast<std::size_t>(r);
            scanning = false;
            apply(ConnectEvent::ScanDone);
        } else if (r == WIFI_SCAN_FAILED) {
            scanCount = 0;
            scanning = false;
            apply(ConnectEvent::ScanDone);
        }
    }

    const wl_status_t status = WiFi.status();
    if (state == ConnectState::Connecting) {
        if (status == WL_CONNECTED) {
            apply(ConnectEvent::LinkUp);
        } else if (status == WL_CONNECT_FAILED) {
            apply(ConnectEvent::WrongPassword);
        } else if (status == WL_NO_SSID_AVAIL) {
            apply(ConnectEvent::NotFound);
        } else if (millis() - connectStartMs > kConnectTimeoutMs) {
            apply(ConnectEvent::Timeout);
        }
    } else if (state == ConnectState::Connected && status != WL_CONNECTED) {
        apply(ConnectEvent::LinkLost);
        lastDropMs = millis();
    } else if (state == ConnectState::Idle && wifiOn && storedSsid[0] != '\0' &&
               millis() - lastDropMs > kRetryMs) {
        startConnect();
    }
}

bool netWifiEnabled() {
    return wifiOn;
}

void netSetWifiEnabled(bool enabled) {
    wifiOn = enabled;
    save();
    if (enabled) {
        WiFi.mode(WIFI_STA);
        startConnect();
    } else {
        WiFi.disconnect(true, false);
        WiFi.mode(WIFI_OFF);
        scanning = false;
        scanCount = 0;
        state = ConnectState::Idle;
    }
}

void netScanStart() {
    if (!wifiOn || scanning) {
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.scanDelete();
    scanCount = 0;
    scanning = WiFi.scanNetworks(true, false) == WIFI_SCAN_RUNNING;
    if (scanning) {
        apply(ConnectEvent::ScanStarted);
    }
}

bool netScanning() {
    return scanning;
}

std::size_t netScanCount() {
    return scanCount;
}

NetworkInfo netScanResult(std::size_t index) {
    NetworkInfo info{};
    if (index >= scanCount) {
        return info;
    }
    const auto i = static_cast<int>(index);
    std::strncpy(info.ssid, WiFi.SSID(i).c_str(), sizeof info.ssid - 1);
    info.rssi = WiFi.RSSI(i);
    info.secured = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    return info;
}

bool netConnect(const char* ssid, const char* passphrase) {
    if (!qpcr::net::validSsid(ssid) || !qpcr::net::validPassphrase(passphrase)) {
        return false;
    }
    std::strncpy(storedSsid, ssid, sizeof storedSsid - 1);
    std::strncpy(storedPass, passphrase, sizeof storedPass - 1);
    save();
    WiFi.disconnect(false, false);
    startConnect();
    return true;
}

void netForget() {
    storedSsid[0] = '\0';
    storedPass[0] = '\0';
    save();
    WiFi.disconnect(true, false);
    apply(ConnectEvent::Forget);
}

ConnectState netState() {
    return state;
}

const char* netSsid() {
    return storedSsid;
}

void netIp(char* out, std::size_t capacity) {
    if (out == nullptr || capacity == 0) {
        return;
    }
    if (state == ConnectState::Connected) {
        std::strncpy(out, WiFi.localIP().toString().c_str(), capacity - 1);
        out[capacity - 1] = '\0';
    } else {
        std::strncpy(out, "-", capacity);
        out[capacity - 1] = '\0';
    }
}

int netRssi() {
    return state == ConnectState::Connected ? WiFi.RSSI() : 0;
}

bool netBluetoothEnabled() {
    return btOn;
}

void netSetBluetoothEnabled(bool enabled) {
    btOn = enabled;
    save();
    // The BLE radio on the ESP32-C6 is not driven by this build (see #37); the setting is
    // persisted so the status bar and a later BLE service follow it.
}

bool netBluetoothSupported() {
    return false;
}

} // namespace qpcr::target
