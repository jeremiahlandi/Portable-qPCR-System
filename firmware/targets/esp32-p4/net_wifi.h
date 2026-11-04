#pragma once

#include <cstddef>
#include <cstdint>

#include "core/net/WifiSettings.h"

// Wi-Fi and Bluetooth settings on the target (issue #37): the ESP32-C6 radio through the
// Arduino WiFi library, credentials and switches in NVS. Plain types only, so the screens
// that use it compile on the host.

namespace qpcr::target {

struct NetworkInfo {
    char ssid[qpcr::net::kSsidMax + 1];
    int rssi;
    bool secured;
};

/// Loads settings from NVS and, when Wi-Fi is on and a network is stored, starts connecting.
void netInit();
/// Drives the scan and connect state machines; call from loop().
void netPoll();

bool netWifiEnabled();
void netSetWifiEnabled(bool enabled);

/// Starts an asynchronous scan; results arrive through netPoll().
void netScanStart();
bool netScanning();
std::size_t netScanCount();
NetworkInfo netScanResult(std::size_t index);

/// Stores the credentials and starts connecting. Rejects invalid input (returns false).
bool netConnect(const char* ssid, const char* passphrase);
/// Drops the link and clears the stored network.
void netForget();

qpcr::net::ConnectState netState();
const char* netSsid();
/// Dotted IPv4 of the station, or "-" when not connected.
void netIp(char* out, std::size_t capacity);
int netRssi();

bool netBluetoothEnabled();
void netSetBluetoothEnabled(bool enabled);
/// False on builds where the BLE radio is not available through the C6 link.
bool netBluetoothSupported();

} // namespace qpcr::target
