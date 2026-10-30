#pragma once

#include "core/hal/Clock.h"

namespace qpcr::fakes {

class FakeClock : public hal::Clock {
public:
    [[nodiscard]] std::uint32_t millis() const override { return now; }
    void advance(std::uint32_t ms) { now += ms; }

    std::uint32_t now = 0;
};

}  // namespace qpcr::fakes
