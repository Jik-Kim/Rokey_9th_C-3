#!/bin/bash
# set_conveyor.py 를 Isaac Sim 기동 없이 돌린다.
#
# isaacsim/python.sh 는 pxr 을 못 읽는다. Isaac Sim 표준 워크플로에서는
# SimulationApp() 을 먼저 띄워야 pxr 이 import 되는데, USD 파일만 고칠
# 거라면 커널 통째로 띄울 이유가 없다. USD 라이브러리 경로만 직접 잡아준다.
set -e

ISAAC=/home/rokey/isaacsim
USDLIBS=$(ls -d "$ISAAC"/extscache/omni.usd.libs-*/ | head -1)

export PYTHONPATH="$USDLIBS"
export LD_LIBRARY_PATH="$USDLIBS/bin:$ISAAC/kit"

exec "$ISAAC/kit/python/bin/python3" \
    "$(dirname "$(readlink -f "$0")")/set_conveyor_offline.py" "$@"
