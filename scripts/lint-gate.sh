#!/usr/bin/env bash
# Same checks as the CI "Build" workflow (.github/workflows/build.yml):
# host unit tests, a build of every firmware environment, and static analysis.
set -euo pipefail

root=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
cd "$root"

if ! command -v pio >/dev/null 2>&1; then
  echo "lint-gate: PlatformIO not found. Install with: uv tool install platformio  (or pip install platformio)" >&2
  exit 1
fi

echo "lint-gate: pio test -e native"
pio test -e native

echo "lint-gate: pio run"
pio run

echo "lint-gate: pio check"
pio check --skip-packages --fail-on-defect=high

echo "lint-gate: OK"
