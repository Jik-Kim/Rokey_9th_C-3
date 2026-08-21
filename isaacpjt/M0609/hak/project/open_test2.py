# isaac-sim.sh --exec 로 실행되어 test2.usd 를 연다.
#   cd ~/isaacsim && ./isaac-sim.sh --exec ~/cobot3_ws/isaacpjt/M0609/hak/project/open_test2.py
#
# test2.usd 는 A43 소터를 넣기 전의 원래 라인이다. test1.usd 와 구조가 다르다.
#
#   조각 18개, 전부 translate z = 0.000 (높이 조정 전, 지면 위에 그대로)
#   본선  A06 / A03 / A24  벨트면 1780mm
#   스퍼  A05             벨트면  770mm
#   그 1010mm 차이를 A37 램프 3개가 이어준다
#   소터 없음, 로봇 없음, 팔레트 없음, 엔드스톱 없음, Boxes 없음
#
# 피더는 기본으로 안 붙인다. spawn_feeder.py 는 벨트면 900mm 를 전제로
# 낙하 높이를 잡는데, 여기 본선은 1780mm 라 박스가 벨트 속으로 들어간다.
# 그래도 붙이려면 PL_FEEDER=1 을 준다.
import os
import sys

import omni.usd

HERE = os.path.dirname(os.path.abspath(__file__))
STAGE = os.path.join(HERE, "project_1", "test2.usd")

omni.usd.get_context().open_stage(STAGE)
print(f"[open_test2] 스테이지: {STAGE}")

if os.environ.get("PL_FEEDER"):
    sys.path.insert(0, HERE)
    import spawn_feeder
    print("[open_test2] 경고 — 이 씬은 본선 벨트면이 1780mm 다. "
          "피더는 900mm 를 전제로 하므로 낙하 위치가 안 맞는다.")
    spawn_feeder.start()
else:
    print("[open_test2] 피더 안 붙임 (붙이려면 PL_FEEDER=1). "
          "본선 1780mm / 스퍼 770mm 두 층 구조라 900mm 전제와 안 맞는다.")
