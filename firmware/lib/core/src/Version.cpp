#include "core/Version.h"

namespace qpcr {

namespace {
#ifdef FIRMWARE_VERSION
constexpr const char* kFirmwareVersion = FIRMWARE_VERSION;
#else
constexpr const char* kFirmwareVersion = "0.0.0-unversioned";
#endif
#ifdef FIRMWARE_BUILD
constexpr const char* kFirmwareBuild = FIRMWARE_BUILD;
#else
constexpr const char* kFirmwareBuild = kFirmwareVersion;
#endif
} // namespace

const char* firmwareVersion() {
    return kFirmwareVersion;
}

const char* firmwareBuild() {
    return kFirmwareBuild;
}

} // namespace qpcr
