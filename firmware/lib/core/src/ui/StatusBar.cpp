#include "core/ui/StatusBar.h"

#include <array>
#include <iterator>

namespace qpcr::ui {

namespace {

constexpr int kEmptyBelow = 10;
constexpr int kLowBelow = 35;
constexpr int kHalfBelow = 65;
constexpr int kHighBelow = 90;
constexpr std::uint32_t kSecondsPerDay = 24U * 60U * 60U;
constexpr std::size_t kClockLength = 5; // "HH:MM"

} // namespace

BatteryLevel batteryLevel(int percent) {
    if (percent < 0) {
        return BatteryLevel::Unknown;
    }
    if (percent < kEmptyBelow) {
        return BatteryLevel::Empty;
    }
    if (percent < kLowBelow) {
        return BatteryLevel::Low;
    }
    if (percent < kHalfBelow) {
        return BatteryLevel::Half;
    }
    if (percent < kHighBelow) {
        return BatteryLevel::High;
    }
    return BatteryLevel::Full;
}

std::size_t formatClock(char* out, std::size_t capacity, std::uint32_t secondsOfDay) {
    if (out == nullptr || capacity <= kClockLength) {
        if (out != nullptr && capacity > 0) {
            *out = '\0';
        }
        return 0;
    }
    const std::uint32_t wrapped = secondsOfDay % kSecondsPerDay;
    const std::uint32_t hours = wrapped / 3600U;
    const std::uint32_t minutes = (wrapped / 60U) % 60U;
    const std::array<char, kClockLength + 1> text{
        static_cast<char>('0' + hours / 10U),   static_cast<char>('0' + hours % 10U),   ':',
        static_cast<char>('0' + minutes / 10U), static_cast<char>('0' + minutes % 10U), '\0'};
    for (std::size_t i = 0; i < text.size(); ++i) {
        *std::next(out, static_cast<std::ptrdiff_t>(i)) = text.at(i);
    }
    return kClockLength;
}

} // namespace qpcr::ui
