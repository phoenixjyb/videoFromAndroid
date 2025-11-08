#!/bin/bash
# Wrapper for the recommended MediaMuxer recorder.
#
# Calls record_on_device.py so CLI users can keep muscle memory with record.sh.
# If you need the legacy WebSocket recorder, run record_video.py directly.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Activate venv if it exists
if [ -d ".venv" ]; then
    source .venv/bin/activate
fi

# Bypass proxy for localhost/127.0.0.1 to avoid connection issues
export NO_PROXY="localhost,127.0.0.1${NO_PROXY:+,$NO_PROXY}"
export no_proxy="localhost,127.0.0.1${no_proxy:+,$no_proxy}"

python3 scripts/record_on_device.py "$@"
