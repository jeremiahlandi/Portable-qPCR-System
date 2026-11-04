#pragma once

#include <cstddef>
#include <cstdint>

// Status bar model (issue #11): the strip along the top of every screen with the clock on the
// left and connectivity plus battery on the right, like the board's factory launcher. This
// header is the board-agnostic part: what to show. The target decides how to draw it.

namespace qpcr::ui {

/// Battery glyph to show for a charge level.
enum class BatteryLevel : std::uint8_t { Unknown, Empty, Low, Half, High, Full };

/// Everything the status bar displays. Filled by the target from its radios, PMIC and clock.
struct StatusBarState {
    bool wifiConnected = false;
    bool bluetoothConnected = false;
    /// 0..100, or -1 when no gauge is available.
    int batteryPercent = -1;
    bool charging = false;
    /// Seconds since midnight (wall clock) or since boot when no RTC is set.
    std::uint32_t secondsOfDay = 0;
};

/// Picks the battery glyph: Unknown below 0, Empty below 10 %, Low below 35 %, Half below 65 %,
/// High below 90 %, Full otherwise. Values above 100 count as Full.
[[nodiscard]] BatteryLevel batteryLevel(int percent);

/// Writes "HH:MM" (24-hour, zero-padded, wrapping at 24 h) into `out`, always NUL-terminated.
/// Returns the characters written, excluding the terminator; 0 if `capacity` is too small.
std::size_t formatClock(char* out, std::size_t capacity, std::uint32_t secondsOfDay);

} // namespace qpcr::ui
