#pragma once

#include "core/hal/DigitalOutput.h"

namespace qpcr::fakes {

class FakeDigitalOutput : public hal::DigitalOutput {
public:
    void set(bool on) override {
        state = on;
        ++setCount;
    }
    [[nodiscard]] bool isOn() const override { return state; }

    bool state = false;
    unsigned setCount = 0;
};

}  // namespace qpcr::fakes
