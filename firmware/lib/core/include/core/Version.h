#pragma once

namespace qpcr {

/// Release number from version.txt, embedded at build time (see scripts/version_flags.py).
/// Always a plain "major.minor.patch".
[[nodiscard]] const char* firmwareVersion();

/// The release number plus SemVer build metadata for a build past the last release tag:
/// "1.1.0+3.gabc1234" (three commits after v1.1.0), ".dirty" appended for an uncommitted
/// tree. Equal to firmwareVersion() at an exact tag. This is what the screen shows.
[[nodiscard]] const char* firmwareBuild();

} // namespace qpcr
