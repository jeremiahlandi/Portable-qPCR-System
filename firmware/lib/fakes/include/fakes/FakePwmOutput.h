#pragma once

#include "core/hal/PwmOutput.h"

namespace qpcr::fakes {

class FakePwmOutput : public hal::PwmOutput {
public:
    void setDuty(float value) override {
        if (value < 0.0F) {
            value = 0.0F;
        } else if (value > 1.0F) {
            value = 1.0F;
        }
        current = value;
        ++setCount;
    }

    [[nodiscard]] float duty() const override { return current; }

    float current = 0.0F;
    unsigned setCount = 0;
};

} // namespace qpcr::fakes
