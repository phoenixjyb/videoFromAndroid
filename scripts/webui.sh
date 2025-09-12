#!/usr/bin/env bash
set -euo pipefail

# Serve the Web UI and optionally forward the device WebSocket port over USB.

PORT=${PORT:-9090}          # WebSocket port on device
SERVE_PORT=${SERVE_PORT:-8000}  # Local HTTP port to serve index.html
FORWARD=true
OPEN_BROWSER=true

usage() {
  cat <<USAGE
Usage: $0 [--no-forward] [--no-open] [--port N] [--serve-port N]

Options:
  --no-forward    Do not set up 'adb forward tcp:<PORT> tcp:<PORT>'
  --no-open       Do not open the browser automatically
  --port N        Device WebSocket port (default: 9090)
  --serve-port N  Local HTTP server port (default: 8000)
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-forward) FORWARD=false; shift ;;
    --no-open) OPEN_BROWSER=false; shift ;;
    --port) PORT=${2:-8080}; shift 2 ;;
    --serve-port) SERVE_PORT=${2:-8000}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 1 ;;
  esac
done

if $FORWARD; then
  if ! command -v adb >/dev/null 2>&1; then
    echo "adb not found in PATH" >&2
    exit 1
  fi
  echo "Forwarding host tcp:${PORT} -> device tcp:${PORT}"
  adb forward tcp:${PORT} tcp:${PORT}
fi

url="http://127.0.0.1:${SERVE_PORT}/index.html?host=127.0.0.1"
echo "Serving web UI at ${url} (Ctrl+C to stop)"

# Prefer python3 for a quick static server
if command -v python3 >/dev/null 2>&1; then
  srv_cmd=(python3 -m http.server "${SERVE_PORT}")
else
  echo "python3 not found; attempting 'python'"
  srv_cmd=(python -m SimpleHTTPServer "${SERVE_PORT}")
fi

if $OPEN_BROWSER; then
  if command -v open >/dev/null 2>&1; then
    open "$url" || true
  fi
fi

"${srv_cmd[@]}"
