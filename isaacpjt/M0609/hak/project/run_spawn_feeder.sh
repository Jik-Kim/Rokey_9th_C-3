#!/bin/bash
# test1.usd 를 열고 투입구 피더를 붙인다. Play 를 누르면 박스가 하나씩 떨어진다.
#   ./run_spawn_feeder.sh
#   PL_N=20 PL_INTERVAL=2 PL_SPAWN=feeder ./run_spawn_feeder.sh
#   PL_CAM_SPEED=2 ./run_spawn_feeder.sh        # 이번 실행만 카메라 속도 지정
set -e

# 카메라 속도는 GUI(Preferences > Navigation, 또는 뷰포트에서 우클릭+휠)에서
# 조절하면 종료할 때 user.config.json 에 저장되어 다음 실행에도 유지된다.
# 그래서 여기서는 값을 강제하지 않는다. PL_CAM_SPEED 를 준 경우에만 덮어쓴다.
#   Isaac 기본값 0.05 m 는 탁상 규모 기준이라 25m 짜리 이 씬에서는 너무 느리다.
CAM_ARG=()
if [ -n "$PL_CAM_SPEED" ]; then
    CAM_ARG=(--/persistent/app/viewport/camMoveVelocity="$PL_CAM_SPEED")
fi

exec /home/rokey/isaacsim/isaac-sim.sh \
    "${CAM_ARG[@]}" \
    --exec "$(dirname "$(readlink -f "$0")")/spawn_feeder.py" "$@"
