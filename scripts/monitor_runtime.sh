#!/usr/bin/env bash
set -euo pipefail

PROCESS_REGEX="${OCS_PROCESS_REGEX:-OpenCarreraSimulator}"
METRICS_FILE="${OCS_METRICS_FILE:-/tmp/opencarrera_metrics.csv}"
SESSION="${OCS_MONITOR_SESSION:-ocs-monitor}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

missing=()
for tool in tmux nvtop pidstat; do
    command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
done

if ((${#missing[@]} > 0)); then
    echo "Missing tools: ${missing[*]}"
    echo "Ubuntu/Debian: sudo apt install tmux nvtop sysstat"
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    exec tmux attach-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n monitor "nvtop"

tmux split-window -v -t "$SESSION:0" \
    "pidstat -u -r -d -w -G '$PROCESS_REGEX' 1"

tmux split-window -h -t "$SESSION:0.1" \
    "bash '$SCRIPT_DIR/metrics_latest.sh' '$METRICS_FILE'"

tmux split-window -h -t "$SESSION:0.0" \
    "bash -lc 'while true; do clear; echo Host summary; echo; uptime; echo; free -h; if command -v sensors >/dev/null 2>&1; then echo; sensors; fi; sleep 2; done'"

tmux select-layout -t "$SESSION:0" tiled
exec tmux attach-session -t "$SESSION"
