#pragma once

#include <cstddef>
#include <cstdint>

// Wi-Fi settings model (issue #37): what the operator may enter and how the connect flow moves.
// Board-agnostic; the target supplies the radio.

namespace qpcr::net {

constexpr std::size_t kSsidMax = 32;      // bytes, IEEE 802.11
constexpr std::size_t kPassphraseMin = 8; // WPA2/WPA3-Personal
constexpr std::size_t kPassphraseMax = 63;

/// 1..32 bytes, no control characters.
[[nodiscard]] bool validSsid(const char* ssid);
/// Empty (open network) or 8..63 printable ASCII bytes.
[[nodiscard]] bool validPassphrase(const char* passphrase);

enum class ConnectState : std::uint8_t {
    Idle,
    Scanning,
    Selecting,
    EnteringPassword,
    Connecting,
    Connected,
    FailedWrongPassword,
    FailedNotFound,
    FailedTimeout,
};

enum class ConnectEvent : std::uint8_t {
    ScanStarted,
    ScanDone,
    ChoseOpenNetwork,
    ChoseSecuredNetwork,
    PasswordEntered,
    Cancel,
    LinkUp,
    WrongPassword,
    NotFound,
    Timeout,
    LinkLost,
    Forget,
};

/// Next state for an event; events that make no sense in a state leave it unchanged.
[[nodiscard]] ConnectState nextState(ConnectState state, ConnectEvent event);

/// True for the three Failed states.
[[nodiscard]] bool isFailure(ConnectState state);

/// Operator-facing text for a state ("Connected", "Wrong password", ...).
[[nodiscard]] const char* stateText(ConnectState state);

} // namespace qpcr::net
