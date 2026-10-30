#pragma once

namespace qpcr {

/// Firmware version from version.txt, embedded at build time (see scripts/version_flags.py).
[[nodiscard]] const char* firmwareVersion();

} // namespace qpcr

#define QA_BAD_CONSTANT 42
