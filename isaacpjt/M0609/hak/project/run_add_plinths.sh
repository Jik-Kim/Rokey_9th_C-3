#!/usr/bin/env bash
set -euo pipefail
exec /home/rokey/isaacsim/python.sh \
    "$(dirname "$(readlink -f "$0")")/add_plinths.py" "$@"
