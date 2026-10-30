#!/usr/bin/env bash
# Same checks as the CI "Build" workflow (.github/workflows/build.yml):
#   1. clang-format  (formatting, firmware sources)
#   2. clang-tidy    (static analysis, board-agnostic core, host flags)
#   3. pio test      (host unit tests under ASan/UBSan, warnings as errors)
#   4. pio run       (every firmware environment)
#   5. pio check     (cppcheck over firmware sources)
# Run before every push. See .cursor/rules/pre-pr-quality-gate.mdc.
set -euo pipefail

root=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
cd "$root"

need() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "lint-gate: '$1' not found. $2" >&2
    exit 1
  fi
}
need pio "Install with: uv tool install platformio   (or: pip install platformio)"
need clang-format "Install LLVM 18+: apt install clang-format clang-tidy   (brew install llvm on macOS)"
need clang-tidy "Install LLVM 18+: apt install clang-format clang-tidy   (brew install llvm on macOS)"

mapfile -t sources < <(git ls-files 'firmware/**/*.cpp' 'firmware/**/*.h' 'firmware/**/*.hpp')
mapfile -t core_sources < <(git ls-files 'firmware/lib/**/*.cpp' 'firmware/lib/**/*.h')

echo "lint-gate: clang-format (${#sources[@]} files)"
clang-format --dry-run -Werror "${sources[@]}"

echo "lint-gate: clang-tidy (${#core_sources[@]} core files)"
clang-tidy --quiet "${core_sources[@]}" -- -std=gnu++17 -x c++ \
  -Ifirmware/lib/core/include -Ifirmware/lib/fakes/include

echo "lint-gate: pio test -e native"
pio test -e native

echo "lint-gate: pio run"
pio run

echo "lint-gate: pio check"
pio check --fail-on-defect=high --fail-on-defect=medium

echo "lint-gate: OK"
