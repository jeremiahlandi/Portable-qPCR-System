#include "time_sync.h"

#include <Arduino.h>
#include <Preferences.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include "core/time/TimeZones.h"

#include "net_wifi.h"

namespace qpcr::target {

namespace {

constexpr const char* kNamespace = "time";
constexpr time_t kSyncedAfter = 1700000000; // 2023-11-14: anything earlier is the unset clock
constexpr const char* kNtp1 = "pool.ntp.org";
constexpr const char* kNtp2 = "time.nist.gov";
constexpr const char* kNtp3 = "time.google.com";

Preferences prefs;
std::size_t zone = qpcr::time::kDefaultTimeZone;
bool requested = false;

void applyZone() {
    setenv("TZ", qpcr::time::kTimeZones.at(zone).posix, 1);
    tzset();
}

bool localNow(struct tm& out) {
    const time_t now = time(nullptr);
    if (now < kSyncedAfter) {
        return false;
    }
    localtime_r(&now, &out);
    return true;
}

} // namespace

void timeInit() {
    prefs.begin(kNamespace, false);
    char id[32] = {};
    prefs.getString("tz", id, sizeof id);
    zone = qpcr::time::timeZoneIndex(id);
    applyZone();
}

void timePoll() {
    if (requested || netState() != qpcr::net::ConnectState::Connected) {
        return;
    }
    configTzTime(qpcr::time::kTimeZones.at(zone).posix, kNtp1, kNtp2, kNtp3);
    requested = true;
}

std::size_t timeZoneSelected() {
    return zone;
}

void timeSetZone(std::size_t index) {
    if (index >= qpcr::time::kTimeZones.size()) {
        return;
    }
    zone = index;
    prefs.putString("tz", qpcr::time::kTimeZones.at(zone).id);
    applyZone();
    timeResync();
}

bool timeSynced() {
    return time(nullptr) >= kSyncedAfter;
}

std::uint32_t timeSecondsOfDay() {
    struct tm t {};
    if (!localNow(t)) {
        return millis() / 1000U;
    }
    return qpcr::time::secondsOfDay(t.tm_hour, t.tm_min, t.tm_sec);
}

void timeClockText(char* out, std::size_t capacity) {
    struct tm t {};
    if (!localNow(t)) {
        std::strncpy(out, "--:--:--", capacity);
        out[capacity - 1] = '\0';
        return;
    }
    std::strftime(out, capacity, "%H:%M:%S", &t);
}

void timeDateText(char* out, std::size_t capacity) {
    struct tm t {};
    if (!localNow(t)) {
        std::strncpy(out, "not set", capacity);
        out[capacity - 1] = '\0';
        return;
    }
    std::strftime(out, capacity, "%Y-%m-%d", &t);
}

void timeResync() {
    requested = false;
}

} // namespace qpcr::target
