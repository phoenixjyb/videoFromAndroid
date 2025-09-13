#!/usr/bin/env bash
set -euo pipefail

# End-to-end quality test:
# - Build + install app
# - ADB forward + launch
# - Set profile and bitrate
# - Capture short H.264 sample
# - Remux to MP4 (if ffmpeg present)
# - Open MP4 (macOS) and print sizes

HOST=${HOST:-127.0.0.1}
PORT=${PORT:-9090}
PROFILE=${PROFILE:-1920x1080x30x0}   # WxHxFPSxHS
MBPS=${MBPS:-12}                     # target bitrate in Mbps
SECONDS=${SECONDS:-6}
OPEN=${OPEN:-1}

echo "[Quality Test] Host=${HOST} Port=${PORT} Profile=${PROFILE} Bitrate=${MBPS}Mbps Duration=${SECONDS}s"

echo "[1/6] Build + install"
./gradlew installDebug >/dev/null

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

echo "[4/6] Apply profile + bitrate"
source .venv/bin/activate 2>/dev/null || true
python scripts/ws_cmd.py --host ${HOST} --port ${PORT} set-profile ${PROFILE} || true
sleep 0.5
bps=$(( MBPS * 1000000 ))
python scripts/ws_cmd.py --host ${HOST} --port ${PORT} set-bitrate ${bps} || true
sleep 1

echo "[5/6] Capture ${SECONDS}s"
mkdir -p captures
ts=$(date +%Y%m%d_%H%M%S)
raw="captures/qtest_${PROFILE}_${MBPS}Mbps_${SECONDS}s_${ts}.h264"
mp4="captures/qtest_${PROFILE}_${MBPS}Mbps_${SECONDS}s_${ts}.mp4"
python scripts/ws_save_h264.py --host ${HOST} --port ${PORT} --seconds ${SECONDS} --out "${raw}"

echo "[6/6] Summarize + remux"
ls -lh "${raw}" | awk '{print "H264:",$5,$9}'
if command -v ffmpeg >/dev/null 2>&1; then
  ffmpeg -y -loglevel error -f h264 -i "${raw}" -c copy "${mp4}" && ls -lh "${mp4}" | awk '{print "MP4 :",$5,$9}'
  if [[ "${OPEN}" == "1" ]] && command -v open >/dev/null 2>&1; then open "${mp4}" || true; fi
else
  echo "ffmpeg not found; skipping MP4 remux"
fi

echo "Done. Files in ./captures"

