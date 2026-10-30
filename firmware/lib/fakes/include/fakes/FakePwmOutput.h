#pragma once

#include "core/hal/PwmOutput.h"

namespace qpcr::fakes {

class FakePwmOutput : public hal::PwmOutput {
public:
    void setDuty(float duty) override {
        if (duty < 0.0F) {
            duty = 0.0F;
        } else if (duty > 1.0F) {
            duty = 1.0F;
        }
        current = duty;
        ++setCount;
    }

    [[nodiscard]] float duty() const override { return current; }

    float current = 0.0F;
    unsigned setCount = 0;
};

}  // namespace qpcr::fakes
