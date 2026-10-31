#include "core/ui/VersionBadge.h"

#include <algorithm>
#include <iterator>
#include <string_view>

namespace qpcr::ui {

namespace {

constexpr std::string_view kFallback = "v?";

} // namespace

std::size_t versionBadgeLabel(char* out, std::size_t capacity, const char* version) {
    if (out == nullptr || capacity == 0) {
        return 0;
    }
    const std::string_view source =
        (version == nullptr || *version == '\0') ? kFallback : std::string_view{version};
    const std::size_t limit = capacity - 1;
    std::size_t written = 0;
    if (source.front() != 'v' && written < limit) {
        *out = 'v';
        written = 1;
    }
    const std::size_t count = std::min(source.size(), limit - written);
    std::copy_n(source.data(), count, std::next(out, static_cast<std::ptrdiff_t>(written)));
    written += count;
    *std::next(out, static_cast<std::ptrdiff_t>(written)) = '\0';
    return written;
}

Point versionBadgeOrigin(int screenWidth, int screenHeight, TextMetrics text, int margin) {
    const int x = screenWidth - margin - text.width;
    const int y = screenHeight - margin - text.height;
    return Point{std::max(x, 0), std::max(y, 0)};
}

} // namespace qpcr::ui
