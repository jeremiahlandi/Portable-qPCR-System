#pragma once

#include <cstddef>
#include <cstdint>

// Wall-clock time on the target (issue #37): zone from NVS, time from SNTP once Wi-Fi is up.
// Until the first sync the clock counts from boot. Plain types only.

namespace qpcr::target {

/// Loads the zone from NVS and applies it to the C library.
void timeInit();
/// Requests an SNTP sync when the link is up; call from loop().
void timePoll();

/// Index into qpcr::time::kTimeZones.
std::size_t timeZoneSelected();
void timeSetZone(std::size_t index);

/// True once SNTP has set the clock.
bool timeSynced();
/// Seconds since local midnight when synced; seconds since boot otherwise.
std::uint32_t timeSecondsOfDay();
/// "HH:MM:SS" local time, or "--:--:--" when not synced.
void timeClockText(char* out, std::size_t capacity);
/// "YYYY-MM-DD" local date, or "not set" when not synced.
void timeDateText(char* out, std::size_t capacity);
/// Forces a new SNTP request on the next poll.
void timeResync();

} // namespace qpcr::target
