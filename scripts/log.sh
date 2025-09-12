#!/usr/bin/env bash
set -euo pipefail

# Focused Logcat capture for camControl
# Defaults: filter by camera app tags; write into logs/ with timestamped filename.

APP_PKG=${APP_PKG:-com.example.camcontrol}
declare -a TAGS=(MainActivity CamControlService Camera2Controller VideoEncoder ControlServer VideoRecorder)

MODE="tags"       # tags | pid | both
CLEAR=true         # clear log buffer before starting
FORWARD=false      # run adb forward for port 9090 (device -> host access)
OUTFILE=""        # custom output file path; default to logs/camcontrol_yyyyMMdd_HHmmss.log

usage() {
  cat <<USAGE
Usage: $0 [--tags|--pid|--both] [--no-clear] [--forward] [--outfile PATH]

Options:
  --tags        Filter by known tags only (default)
  --pid         Filter by app process only
  --both        Filter by pid AND tags (extra strict)
  --no-clear    Do not clear log buffer before capture
  --forward     Run 'adb forward tcp:9090 tcp:9090' (USB access to device WS)
  --outfile P   Write to custom path (default logs/camcontrol_<ts>.log)

Env:
  APP_PKG       Package name (default: com.example.camcontrol)
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tags) MODE="tags"; shift ;;
    --pid) MODE="pid"; shift ;;
    --both) MODE="both"; shift ;;
    --no-clear) CLEAR=false; shift ;;
    --forward) FORWARD=true; shift ;;
    --reverse) echo "--reverse is deprecated; using --forward for device-local server"; FORWARD=true; shift ;;
    --outfile) OUTFILE=${2:-}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 1 ;;
  esac
done

if ! command -v adb >/dev/null 2>&1; then
  echo "adb not found in PATH" >&2
  exit 1
fi

mkdir -p logs
ts=$(date +%Y%m%d_%H%M%S)
outfile_default="logs/camcontrol_${ts}.log"
outfile="${OUTFILE:-$outfile_default}"

if $FORWARD; then
  echo "Setting up adb forward tcp:9090 -> tcp:9090"
  adb forward tcp:9090 tcp:9090 || true
fi

if $CLEAR; then
  echo "Clearing logcat buffer"
  adb logcat -c || true
fi

base=(adb logcat -v time)

add_tag_filters() {
  local args=("-s")
  for t in "${TAGS[@]}"; do
    args+=("${t}:V")
  done
  printf '%s\n' "${args[@]}"
}

pid=""
if [[ "$MODE" == "pid" || "$MODE" == "both" ]]; then
  echo "Resolving PID for ${APP_PKG} (will wait up to 10s if not running)"
  for i in {1..20}; do
    pid=$(adb shell pidof -s "$APP_PKG" | tr -d '\r') || true
    [[ -n "$pid" ]] && break
    sleep 0.5
  done
  if [[ -z "$pid" ]]; then
    echo "Warning: app not running; PID filter will be skipped until app starts" >&2
  else
    echo "Using PID: $pid"
  fi
fi

cmd=("${base[@]}")
case "$MODE" in
  tags)
    cmd+=( $(add_tag_filters) ) ;;
  pid)
    [[ -n "$pid" ]] && cmd+=("--pid=$pid") ;;
  both)
    [[ -n "$pid" ]] && cmd+=("--pid=$pid")
    cmd+=( $(add_tag_filters) ) ;;
esac

echo "Writing logs to $outfile"
printf 'Command: %q ' "${cmd[@]}"; echo
"${cmd[@]}" | tee "$outfile"
