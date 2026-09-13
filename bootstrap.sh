#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-OpenCarreraSimulator}"

mkdir -p "$ROOT"/{cmake,scripts,app/src,tests/{core,math},engine/{core/{include/ocs/core,src},math/{include/ocs/math,src},platform/{include/ocs/platform,src},render/{include/ocs/render,src/vulkan}}}

echo "OpenCarreraSimulator Step 2 directories created at: $ROOT"
