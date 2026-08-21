"""이미 열려 있는 Isaac Sim 세션에 피더+소터를 붙인다.

Script Editor 에 이 세 줄을 붙여넣어라. 파일을 File>Open 으로 직접 열었거나
open_test1.py 를 안 거쳤으면 피더가 로드되지 않아 Play 를 눌러도 박스가
안 나온다. "랜덤 스폰이 안 된다" 는 대부분 이것이다.

    import sys; sys.path.insert(0, "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project")
    import attach; attach.go()

주의: 스크립트를 오프라인에서 고친 뒤라면 stage 를 먼저 다시 열어야 한다.
GUI 가 들고 있는 스테이지는 디스크 변경을 따라오지 않고, Ctrl+S 하면
오프라인 수정본을 덮어쓴다.
"""

import importlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STAGE = os.path.join(HERE, "project_1", "test1.usd")


def go(reopen=False):
    if HERE not in sys.path:
        sys.path.insert(0, HERE)

    import omni.usd
    ctx = omni.usd.get_context()

    if reopen:
        ctx.open_stage(STAGE)
        print(f"[attach] 스테이지 재오픈: {STAGE}")

    stage = ctx.get_stage()
    print(f"[attach] 현재 스테이지: {stage.GetRootLayer().identifier}")

    import spawn_feeder
    import sorter
    importlib.reload(spawn_feeder)      # 오프라인에서 고친 내용을 반영한다
    importlib.reload(sorter)

    try:
        spawn_feeder.stop()
    except Exception:
        pass
    try:
        sorter.stop()
    except Exception:
        pass

    spawn_feeder.start()
    sorter.start()
    print("[attach] 부착 완료. Play 를 눌러라.")


def check():
    """왜 안 나오는지 짚어준다."""
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    ident = stage.GetRootLayer().identifier
    print(f"스테이지      : {ident}")
    print(f"디스크와 일치 : {os.path.samefile(ident, STAGE) if os.path.exists(ident) else '?'}")
    for path in ("/World/Cube_01", "/World/ConveyorTrack", "/World/Boxes"):
        p = stage.GetPrimAtPath(path)
        print(f"{path:<24} {'있음' if p else '없음'}"
              + (f"  자식 {len(p.GetChildren())}개" if p and path == "/World/Boxes" else ""))
    import spawn_feeder
    print(f"피더 동작 중  : {bool(spawn_feeder._state)}")
