#!/usr/bin/env bash
set -euo pipefail

PRESET="${1:-gcc-debug}"
cmake --preset "$PRESET"
cmake --build --preset "$PRESET"
