#include "core/Version.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "0.0.0-unversioned"
#endif

namespace qpcr {

const char* firmwareVersion() { return FIRMWARE_VERSION; }

}  // namespace qpcr
