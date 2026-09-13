#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-.}"
cd "$ROOT"

# Step 2 already uses ocs/OpenCarreraSimulator naming. This helper removes
# the old Step 1 rs include trees after you have copied the Step 2 files.
rm -rf engine/core/include/rs engine/math/include/rs engine/platform/include/rs

echo "Removed legacy rs/ headers. Step 2 sources use namespace ocs."
