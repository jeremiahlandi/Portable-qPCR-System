#pragma once

#include <cstddef>

// Version badge: the firmware version shown in the bottom-right corner of every screen
// (issue #28). Board-agnostic: the target supplies the display size and the text metrics
// of its font, this header supplies the label text and where to draw it.

namespace qpcr::ui {

/// Pixels between the label's bottom-right corner and the display's bottom-right corner.
constexpr int kVersionBadgeMargin = 4;

/// Top-left point, in display pixels, where text is drawn.
struct Point {
    int x;
    int y;
};

/// Rendered size of a text string in the target's current font.
struct TextMetrics {
    int width;
    int height;
};

/// Writes the badge text ("v" + version, or the version unchanged if it already starts
/// with "v") into `out`, always NUL-terminated and truncated to `capacity - 1` characters.
/// Returns the number of characters written, excluding the terminator. A null or empty
/// `version` yields "v?" so a missing build flag is visible rather than blank.
std::size_t versionBadgeLabel(char* out, std::size_t capacity, const char* version);

/// Top-left origin that places text of the given metrics `margin` pixels inside the
/// bottom-right corner of a `screenWidth` x `screenHeight` display. Coordinates are clamped
/// at zero so a label wider than the display still starts on screen.
[[nodiscard]] Point versionBadgeOrigin(int screenWidth, int screenHeight, TextMetrics text,
                                       int margin = kVersionBadgeMargin);

} // namespace qpcr::ui
