#!/usr/bin/env bash
set -euo pipefail

# Automate build → install → forward → launch → optional browser open

PORT=${PORT:-9090}
OPEN_BROWSER=${OPEN_BROWSER:-1}
SKIP_BI=${SKIP_BI:-0}   # 1 to skip build+install

if [[ "${SKIP_BI}" != "1" ]]; then
  echo "[1/4] Building + installing debug APK"
  ./gradlew installDebug
else
  echo "[1/4] Skipping build + install (SKIP_BI=1)"
fi

echo "[2/4] ADB forward tcp:${PORT} -> tcp:${PORT}"
adb forward tcp:${PORT} tcp:${PORT} || true

echo "[3/4] Launching app"
adb shell am start -n com.example.camcontrol/.MainActivity >/dev/null || true

if [[ "${OPEN_BROWSER}" == "1" ]]; then
  URL="http://127.0.0.1:${PORT}"
  echo "[4/4] Opening ${URL}"
  if command -v open >/dev/null 2>&1; then open "${URL}" || true; fi
fi

echo "Done. Visit http://127.0.0.1:${PORT}"
