#include "core/time/TimeZones.h"

#include <cstring>

namespace qpcr::time {

std::size_t timeZoneIndex(const char* id) {
    if (id == nullptr) {
        return kDefaultTimeZone;
    }
    for (std::size_t i = 0; i < kTimeZones.size(); ++i) {
        if (std::strcmp(kTimeZones.at(i).id, id) == 0) {
            return i;
        }
    }
    return kDefaultTimeZone;
}

unsigned secondsOfDay(int hour, int minute, int second) {
    constexpr int kDay = 24 * 60 * 60;
    long total = (static_cast<long>(hour) * 3600L) + (static_cast<long>(minute) * 60L) + second;
    total %= kDay;
    if (total < 0) {
        total += kDay;
    }
    return static_cast<unsigned>(total);
}

} // namespace qpcr::time
