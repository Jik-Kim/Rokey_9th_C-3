#!/usr/bin/env bash
set -euo pipefail
ISAAC=/home/rokey/isaacsim
USDLIBS=$(ls -d "$ISAAC"/extscache/omni.usd.libs-*/ | head -1)
export PYTHONPATH="$USDLIBS"
export LD_LIBRARY_PATH="$USDLIBS/bin:$ISAAC/kit"
exec "$ISAAC/kit/python/bin/python3" "$(dirname "$(readlink -f "$0")")/level_sorters.py" "$@"
