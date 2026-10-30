#!/usr/bin/env bash
# Same checks as the CI "Build" job (.github/workflows/build.yml).
set -euo pipefail

root=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
cd "$root"

echo "lint-gate: pio run"
pio run

echo "lint-gate: pio check"
pio check --skip-packages --fail-on-defect=high

echo "lint-gate: OK"
