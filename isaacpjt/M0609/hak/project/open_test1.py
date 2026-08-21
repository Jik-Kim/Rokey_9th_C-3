# isaac-sim.sh --exec 로 실행되어 test1.usd 를 열고 투입구 피더까지 붙인다.
#   cd ~/isaacsim && ./isaac-sim.sh --exec ~/cobot3_ws/isaacpjt/M0609/hak/project/open_test1.py
#
# 피더 없이 스테이지만 보려면 PL_NO_FEEDER=1.
# 색상 분류까지 켜려면 PL_SORT=1 (분기에 받는 벨트를 놓은 뒤에).
# 예전에는 이 스크립트가 스테이지만 열었는데, 그러면 Play 를 눌러도 박스가
# 안 나온다 (피더가 로드되지 않으므로). 스폰이 안 된다는 오해가 여기서 나왔다.
import os
import sys

import omni.usd

STAGE = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
omni.usd.get_context().open_stage(STAGE)

if os.environ.get("PL_NO_FEEDER"):
    print("[open_test1] PL_NO_FEEDER — 피더를 붙이지 않는다.")
else:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import spawn_feeder
    spawn_feeder.start()

    # 기본은 분류 끔. 켜려면 PL_SORT=1.
    # 분기(Rollers_01)가 아직 아무 벨트에도 안 닿아 있다. 분류를 켜면 팝업 휠이
    # 박스를 허공으로 밀어내서 튕기고 막힌다. 분기에 받는 벨트를 놓기 전까지는
    # 꺼두는 게 맞다.
    if not os.environ.get("PL_SORT"):
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        off = []
        for name in ("SorterBlue", "SorterGreen", "SorterRed"):
            a = stage.GetAttributeAtPath(
                f"/World/{name}/Sorter/ActionGraph/binary_switch.inputs:value")
            if a:
                a.Set(False)
                off.append(name)
        print(f"[open_test1] 분류 끔 {off} — 본선 통과만 한다. 켜려면 PL_SORT=1")
    else:
        import sorter
        sorter.start()
        print("[open_test1] 피더 + 색상 소터 부착 완료. Play 를 누르면 투입·분류된다.")
