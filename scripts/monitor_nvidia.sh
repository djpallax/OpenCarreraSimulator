#!/usr/bin/env bash
set -euo pipefail
GPU_INDEX="${1:-0}"
if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "nvidia-smi not found"
    exit 1
fi
exec nvidia-smi dmon -i "$GPU_INDEX" -s pucmt
