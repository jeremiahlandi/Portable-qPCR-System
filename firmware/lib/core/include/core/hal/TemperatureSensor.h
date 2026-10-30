#pragma once

#include <cstdint>

namespace qpcr::hal {

/// Why a temperature reading is not usable. The safety layer (issue #5) acts on these.
enum class SensorFault : std::uint8_t {
    None,
    Open,        ///< sensor disconnected / open circuit
    Short,       ///< sensor shorted
    OutOfRange,  ///< value outside the sensor's physical range
    NotReady,    ///< no conversion completed yet
    Bus,         ///< communication failure with the sensor chip
};

struct TemperatureReading {
    float celsius = 0.0F;
    SensorFault fault = SensorFault::NotReady;

    [[nodiscard]] bool valid() const { return fault == SensorFault::None; }
};

/// A temperature sensor. Implementations must never block the caller.
class TemperatureSensor {
public:
    virtual ~TemperatureSensor() = default;
    /// Latest completed reading. Returns a fault instead of a stale or invalid value.
    [[nodiscard]] virtual TemperatureReading read() = 0;
};

}  // namespace qpcr::hal
