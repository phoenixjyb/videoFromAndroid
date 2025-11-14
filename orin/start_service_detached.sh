#!/bin/bash
# Wrapper to start services completely detached from parent process
# This prevents systemd from killing child processes when restarting the parent

# Detach completely using setsid and redirect all I/O
setsid bash -c "cd $(dirname "$0") && ./start_all_services.sh" </dev/null >/dev/null 2>&1 &

# Exit immediately so parent doesn't wait
exit 0
