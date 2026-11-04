#include "core/net/WifiSettings.h"

#include <algorithm>
#include <string_view>

namespace qpcr::net {

namespace {

bool allPrintable(std::string_view s) {
    return std::all_of(s.begin(), s.end(), [](char c) {
        const auto u = static_cast<unsigned char>(c);
        return u >= 0x20 && u != 0x7F;
    });
}

} // namespace

bool validSsid(const char* ssid) {
    if (ssid == nullptr) {
        return false;
    }
    const std::string_view s{ssid};
    return !s.empty() && s.size() <= kSsidMax && allPrintable(s);
}

bool validPassphrase(const char* passphrase) {
    if (passphrase == nullptr) {
        return false;
    }
    const std::string_view p{passphrase};
    if (p.empty()) {
        return true;
    }
    return p.size() >= kPassphraseMin && p.size() <= kPassphraseMax && allPrintable(p);
}

ConnectState nextState(ConnectState state, ConnectEvent event) {
    switch (event) {
    case ConnectEvent::ScanStarted:
        return (state == ConnectState::Connecting) ? state : ConnectState::Scanning;
    case ConnectEvent::ScanDone:
        return (state == ConnectState::Scanning) ? ConnectState::Selecting : state;
    case ConnectEvent::ChoseOpenNetwork:
        return (state == ConnectState::Selecting) ? ConnectState::Connecting : state;
    case ConnectEvent::ChoseSecuredNetwork:
        return (state == ConnectState::Selecting) ? ConnectState::EnteringPassword : state;
    case ConnectEvent::PasswordEntered:
        return (state == ConnectState::EnteringPassword) ? ConnectState::Connecting : state;
    case ConnectEvent::Cancel:
        return (state == ConnectState::Connected) ? state : ConnectState::Idle;
    case ConnectEvent::LinkUp:
        return ConnectState::Connected;
    case ConnectEvent::WrongPassword:
        return (state == ConnectState::Connecting) ? ConnectState::FailedWrongPassword : state;
    case ConnectEvent::NotFound:
        return (state == ConnectState::Connecting) ? ConnectState::FailedNotFound : state;
    case ConnectEvent::Timeout:
        return (state == ConnectState::Connecting) ? ConnectState::FailedTimeout : state;
    case ConnectEvent::LinkLost:
        return (state == ConnectState::Connected) ? ConnectState::Idle : state;
    case ConnectEvent::Forget:
        return ConnectState::Idle;
    default:
        return state;
    }
}

bool isFailure(ConnectState state) {
    return state == ConnectState::FailedWrongPassword || state == ConnectState::FailedNotFound ||
           state == ConnectState::FailedTimeout;
}

const char* stateText(ConnectState state) {
    switch (state) {
    case ConnectState::Idle:
        return "Not connected";
    case ConnectState::Scanning:
        return "Scanning...";
    case ConnectState::Selecting:
        return "Choose a network";
    case ConnectState::EnteringPassword:
        return "Enter the password";
    case ConnectState::Connecting:
        return "Connecting...";
    case ConnectState::Connected:
        return "Connected";
    case ConnectState::FailedWrongPassword:
        return "Wrong password";
    case ConnectState::FailedNotFound:
        return "Network not found";
    case ConnectState::FailedTimeout:
        return "Timed out";
    default:
        return "?";
    }
}

} // namespace qpcr::net
