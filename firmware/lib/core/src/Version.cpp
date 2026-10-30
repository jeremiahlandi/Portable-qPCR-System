#include "core/Version.h"

namespace qpcr {

namespace {
#ifdef FIRMWARE_VERSION
constexpr const char* kFirmwareVersion = FIRMWARE_VERSION;
#else
constexpr const char* kFirmwareVersion = "0.0.0-unversioned";
#endif
} // namespace

const char* firmwareVersion() {
    return kFirmwareVersion;
}

} // namespace qpcr
