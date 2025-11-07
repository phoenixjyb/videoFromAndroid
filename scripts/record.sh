#!/bin/bash
# Convenience wrapper for record_video.py
# Activates venv and runs the script with proxy bypass for localhost

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

python scripts/record_video.py "$@"
