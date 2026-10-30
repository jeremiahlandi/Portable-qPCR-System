#!/usr/bin/env bash
# Same checks as the CI "Build" workflow (.github/workflows/build.yml):
#   1. clang-format  (formatting, firmware sources)
#   2. clang-tidy    (static analysis, board-agnostic core, host flags)
#   3. cppcheck      (static analysis, all firmware sources)
#   4. replay check  (committed thermal test data matches the Klipper originals)
#   5. pio test      (host unit tests under ASan/UBSan, warnings as errors)
#   6. pio run       (every firmware environment)
# All Python-packaged tools come from uv (pyproject.toml / uv.lock); nothing else to install.
# Run before every push. See .cursor/rules/pre-pr-quality-gate.mdc.
set -euo pipefail

root=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
cd "$root"

if ! command -v uv >/dev/null 2>&1; then
  echo "lint-gate: 'uv' not found. Install it: curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
  exit 1
fi

echo "lint-gate: uv sync --frozen"
uv sync --frozen --quiet

mapfile -t sources < <(git ls-files 'firmware/**/*.cpp' 'firmware/**/*.h' 'firmware/**/*.hpp')
mapfile -t core_sources < <(git ls-files 'firmware/lib/**/*.cpp' 'firmware/lib/**/*.h')

echo "lint-gate: clang-format (${#sources[@]} files)"
uv run clang-format --dry-run -Werror "${sources[@]}"

echo "lint-gate: clang-tidy (${#core_sources[@]} core files)"
uv run clang-tidy --quiet "${core_sources[@]}" -- -std=gnu++17 -x c++ \
  -Ifirmware/lib/core/include -Ifirmware/lib/fakes/include

echo "lint-gate: cppcheck"
uv run cppcheck --enable=warning,style,performance,portability --inline-suppr \
  --suppressions-list=.cppcheck-suppressions --std=c++17 --error-exitcode=1 --quiet \
  -Ifirmware/lib/core/include -Ifirmware/lib/fakes/include \
  firmware/lib firmware/targets firmware/test

if [[ -f firmware/tools/klipper_reference/replay.py ]]; then
  echo "lint-gate: replay data matches the Klipper originals"
  uv run python -I firmware/tools/klipper_reference/replay.py --check
fi

echo "lint-gate: pio test -e native"
uv run pio test -e native

echo "lint-gate: pio run"
uv run pio run

echo "lint-gate: OK"
