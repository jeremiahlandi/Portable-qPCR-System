#!/usr/bin/env bash
# Enable repo pre-push build gate (see .cursor/rules/pre-pr-quality-gate.mdc).
set -euo pipefail
root=$(git rev-parse --show-toplevel)
cd "$root"
chmod +x .githooks/pre-push scripts/lint-gate.sh
git config core.hooksPath .githooks
echo "Git hooksPath set to .githooks (pre-push runs scripts/lint-gate.sh via uv)."
