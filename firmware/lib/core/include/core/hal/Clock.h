#pragma once

#include <cstdint>

namespace qpcr::hal {

/// Monotonic time source. Milliseconds since an arbitrary origin; wraps after ~49 days,
/// so callers must compare differences, never absolute values.
class Clock {
public:
    virtual ~Clock() = default;
    [[nodiscard]] virtual std::uint32_t millis() const = 0;

    /// Elapsed milliseconds since `since`, correct across wrap-around.
    [[nodiscard]] std::uint32_t elapsedSince(std::uint32_t since) const { return millis() - since; }
};

}  // namespace qpcr::hal
