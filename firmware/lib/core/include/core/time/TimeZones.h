#pragma once

#include <array>
#include <cstddef>

// Time zones the operator can pick in Settings (issue #37). The POSIX string is what the
// target hands to the C library so local time and daylight saving come out right; the id is
// what gets persisted, so the table may be reordered without breaking stored settings.

namespace qpcr::time {

struct TimeZone {
    const char* id;
    const char* label;
    const char* posix;
};

constexpr std::array<TimeZone, 8> kTimeZones{{
    {"utc", "UTC", "UTC0"},
    {"us-eastern", "US Eastern", "EST5EDT,M3.2.0,M11.1.0"},
    {"us-central", "US Central", "CST6CDT,M3.2.0,M11.1.0"},
    {"us-mountain", "US Mountain", "MST7MDT,M3.2.0,M11.1.0"},
    {"us-arizona", "US Arizona", "MST7"},
    {"us-pacific", "US Pacific", "PST8PDT,M3.2.0,M11.1.0"},
    {"us-alaska", "US Alaska", "AKST9AKDT,M3.2.0,M11.1.0"},
    {"us-hawaii", "US Hawaii", "HST10"},
}};

constexpr std::size_t kDefaultTimeZone = 1; // US Eastern: Kalix and Landi are in Ohio

/// Index of the zone with this id, or kDefaultTimeZone when unknown or null.
[[nodiscard]] std::size_t timeZoneIndex(const char* id);

/// Seconds since local midnight for a broken-down local time; wraps bad input into range.
[[nodiscard]] unsigned secondsOfDay(int hour, int minute, int second);

} // namespace qpcr::time
