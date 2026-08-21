#!/bin/bash
# fix_column_gaps.py 를 Isaac Sim 기동 없이 돌린다. run_set_conveyor.sh 와 같은 방식.
set -e

ISAAC=/home/rokey/isaacsim
USDLIBS=$(ls -d "$ISAAC"/extscache/omni.usd.libs-*/ | head -1)

export PYTHONPATH="$USDLIBS"
export LD_LIBRARY_PATH="$USDLIBS/bin:$ISAAC/kit"

exec "$ISAAC/kit/python/bin/python3" \
    "$(dirname "$(readlink -f "$0")")/fix_column_gaps.py" "$@"
