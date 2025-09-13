#!/usr/bin/env bash
set -euo pipefail

# End-to-end quality test:
# - Build + install app
# - ADB forward + launch
# - For each requested bitrate: set profile + bitrate, capture sample, remux, open

usage() {
  cat <<USAGE
Usage: $0 [--host IP] [--port N] [--profile WxHxFPSxHS] [--seconds N] [--open 0|1] [--mbps N] [--12] [--40] [--no-build-install]

Examples:
  $0 --12                   # 1080p30 @ 12 Mbps, 6s (defaults)
  $0 --12 --40              # Run two captures back-to-back at 12 and 40 Mbps
  HOST=192.168.1.50 $0 --mbps 18 --profile 3840x2160x30x0 --seconds 8
USAGE
}

HOST=${HOST:-127.0.0.1}
PORT=${PORT:-9090}
PROFILE=${PROFILE:-1920x1080x30x0}   # WxHxFPSxHS
DURATION=${DURATION:-6}               # Use DURATION to avoid clashing with bash SECONDS
OPEN=${OPEN:-1}
SKIP_BI=${SKIP_BI:-0}                 # 1 to skip build+install

declare -a MBPS_LIST=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host) HOST=${2:-}; shift 2;;
    --port) PORT=${2:-}; shift 2;;
    --profile) PROFILE=${2:-}; shift 2;;
    --seconds) DURATION=${2:-}; shift 2;;
    --open) OPEN=${2:-1}; shift 2;;
    --mbps) MBPS_LIST+=("${2:-}"); shift 2;;
    --[0-9]*) v=${1#--}; MBPS_LIST+=("${v}"); shift;;
    --no-build-install) SKIP_BI=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1" >&2; usage; exit 1;;
  esac
done

if [[ ${#MBPS_LIST[@]} -eq 0 ]]; then
  MBPS_LIST+=("${MBPS:-12}")
fi

echo "[Quality Test] Host=${HOST} Port=${PORT} Profile=${PROFILE} Bitrates=${MBPS_LIST[*]} Mbps Duration=${DURATION}s"

if [[ "${SKIP_BI}" != "1" ]]; then
  echo "[1/6] Build + install"
  ./gradlew installDebug >/dev/null
else
  echo "[1/6] Skipping build + install (--no-build-install)"
fi

echo "[2/6] ADB forward + launch"
adb forward tcp:${PORT} tcp:${PORT} >/dev/null || true
adb shell am start -n com.example.camcontrol/.MainActivity >/dev/null || true

echo "[3/6] Wait for server http://${HOST}:${PORT}/"
for i in {1..30}; do
  if env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY curl -fsS --max-time 1 --noproxy '*' http://${HOST}:${PORT}/ >/dev/null 2>&1; then
    break
  fi
  sleep 0.3
done

source .venv/bin/activate 2>/dev/null || true
python scripts/ws_cmd.py --host ${HOST} --port ${PORT} set-profile ${PROFILE} || true
sleep 0.5

mkdir -p captures
ts=$(date +%Y%m%d_%H%M%S)

for MBPS in "${MBPS_LIST[@]}"; do
  echo "[4/6] Apply bitrate ${MBPS} Mbps"
  bps=$(( MBPS * 1000000 ))
  python scripts/ws_cmd.py --host ${HOST} --port ${PORT} set-bitrate ${bps} || true
  sleep 1

  echo "[5/6] Capture ${DURATION}s @ ${MBPS} Mbps"
  raw="captures/qtest_${PROFILE}_${MBPS}Mbps_${DURATION}s_${ts}.h264"
  mp4="captures/qtest_${PROFILE}_${MBPS}Mbps_${DURATION}s_${ts}.mp4"
  # Allow Ctrl+C to interrupt capture but still finalize below
  python scripts/ws_save_h264.py --host ${HOST} --port ${PORT} --seconds ${DURATION} --out "${raw}" || true

  echo "[6/6] Summarize + remux for ${MBPS} Mbps"
  ls -lh "${raw}" | awk '{print "H264:",$5,$9}'
  if command -v ffmpeg >/dev/null 2>&1; then
    ffmpeg -y -loglevel error -f h264 -i "${raw}" -c copy "${mp4}" && ls -lh "${mp4}" | awk '{print "MP4 :",$5,$9}'
    if [[ "${OPEN}" == "1" ]] && command -v open >/dev/null 2>&1; then open "${mp4}" || true; fi
  else
    echo "ffmpeg not found; skipping MP4 remux"
  fi
done

echo "Done. Files in ./captures"
