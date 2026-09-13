#!/usr/bin/env bash
set -euo pipefail

METRICS_FILE="${1:-${OCS_METRICS_FILE:-/tmp/opencarrera_metrics.csv}}"

while true; do
    clear
    echo "OpenCarreraSimulator engine metrics"
    echo

    if [[ ! -f "$METRICS_FILE" ]]; then
        echo "Waiting for $METRICS_FILE"
        echo "Run the simulator with --metrics"
        sleep 1
        continue
    fi

    line="$(tail -n 1 "$METRICS_FILE")"
    if [[ -z "$line" ]] || [[ "$line" == time_s,* ]]; then
        echo "Waiting for first metrics sample..."
        sleep 1
        continue
    fi

    awk -F',' '
        {
            printf "FPS             %8.2f\n", $3;
            printf "Frame           %8.3f ms\n", $4;
            printf "CPU total       %8.3f ms\n", $5;
            printf "CPU work        %8.3f ms\n", $6;
            printf "CPU sync        %8.3f ms\n", $7;
            printf "Acquire         %8.3f ms\n", $8;
            printf "Present         %8.3f ms\n", $9;
            printf "GPU             %8.3f ms\n", $10;
            printf "Draw calls      %8d\n", $11;
            printf "Triangles       %8d\n", $12;
            printf "Instances       %8d\n", $13;
            printf "Models          %8d\n", $14;
            printf "Meshes          %8d\n", $15;
            printf "Materials       %8d\n", $16;
            printf "Textures        %8d\n", $17;
            printf "Samplers        %8d\n", $18;
            printf "Vertex memory   %8.3f MiB\n", $19;
            printf "Index memory    %8.3f MiB\n", $20;
            printf "Texture memory  %8.3f MiB\n", $21;
        }
    ' <<< "$line"

    echo
    echo "Source: $METRICS_FILE"
    sleep 1
done
