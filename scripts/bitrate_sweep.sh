#!/usr/bin/env bash
set -euo pipefail

# Sweep bitrates, capture short clips, summarize sizes

HOST=${HOST:-127.0.0.1}
PORT=${PORT:-9090}
DURATION=${DURATION:-5}                    # Avoid bash SECONDS collision
BITRATES=${BITRATES:-"6 9 12 16 20 28"}  # Mbps

source .venv/bin/activate 2>/dev/null || true

echo "Sweeping bitrates (Mbps): ${BITRATES}"
mkdir -p captures

for mbps in ${BITRATES}; do
  bps=$(( mbps * 1000000 ))
  echo "==> setBitrate ${mbps} Mbps"
  python scripts/ws_cmd.py --host ${HOST} --port ${PORT} set-bitrate ${bps} || true
  sleep 1
  out="captures/cap_${mbps}Mbps_${DURATION}s.h264"
  echo "    capture -> ${out}"
  python scripts/ws_save_h264.py --host ${HOST} --port ${PORT} --seconds ${DURATION} --out "${out}" || true
done

echo "Summary:"
ls -lh captures/*.h264 | awk '{printf "%8s  %s\n", $5, $9}'
