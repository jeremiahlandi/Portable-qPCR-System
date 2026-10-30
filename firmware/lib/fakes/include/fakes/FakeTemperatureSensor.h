#pragma once

#include "core/hal/TemperatureSensor.h"

namespace qpcr::fakes {

class FakeTemperatureSensor : public hal::TemperatureSensor {
public:
    hal::TemperatureReading read() override {
        ++readCount;
        return reading;
    }

    void set(float celsius) { reading = {celsius, hal::SensorFault::None}; }
    void fail(hal::SensorFault fault) { reading = {0.0F, fault}; }

    hal::TemperatureReading reading{};
    unsigned readCount = 0;
};

}  // namespace qpcr::fakes
