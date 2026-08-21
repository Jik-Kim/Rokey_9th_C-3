#!/usr/bin/env python3
"""색상별로 박스를 분기로 빼낸다. A43 소터의 내장 팝업 휠을 켜고 끈다.

배정 — 먼 곳부터 R, G, B
    3호 빨강 (R)  ->  라인 끝 (상단 _08_B). 소터 OFF 로 통과.
    4호 초록 (G)  ->  SorterGreen  (분기 -> _10)
    5호 파랑 (B)  ->  SorterBlue   (분기 -> _12 -> _04)

제어 방식
  A43 에셋 안의 Sorter/ActionGraph 가 완결된 팝업 휠 기구다.
      binary_switch  ConstantBool   <- ON/OFF
      Direction      -45.0          휠 전환 각도
      SorterSpeed    -3.0
      write_prim_attribute -> 휠 그룹 12개의 xformOp:rotateXYZ
  binary_switch 를 True 로 두면 휠이 돌아가 박스를 분기로 밀어낸다.
  밀대를 만들거나 physics:velocity 를 덮어쓸 필요가 없다. 앞선 두 시도
  (속도 덮어쓰기 -> 박스 뒤집힘, 키네매틱 푸셔 -> 스트로크 부족)를
  이걸로 대체한다.

색 판정
  지금은 프림 이름의 No3/No4/No5 접미사를 읽는다. 정답지를 보는 것이라
  인식이 아니다. 카메라 RGB 판정이 붙으면 _color_of() 만 갈아끼운다.

    import sorter; sorter.start()
"""

import os

import omni.timeline
import omni.usd
from omni.physx import get_physx_interface
from pxr import Usd, UsdGeom

BOX_ROOT = "/World/Boxes"
SWITCH = "/World/{sorter}/Sorter/ActionGraph/binary_switch.inputs:value"

# 색 -> (소터 프림 이름, 라벨). None 이면 분기하지 않는다.
ROUTE = {
    "No3": (None,          "라인 끝 (빨강)"),
    "No4": ("SorterGreen", "중간 분기 (초록)"),
    "No5": ("SorterBlue",  "첫 분기 (파랑)"),
}

# 소터 기구가 본선을 차지하는 y 구간 (에셋 로컬 x 1.161..2.204 + 배치 원점).
# install_sorters.py 의 원점과 맞물린다.
ZONE = {
    "SorterBlue":  (3.488 + 1.161, 3.488 + 2.204),
    "SorterGreen": (7.470 + 1.161, 7.470 + 2.204),
}
LEAD = float(os.environ.get("PL_SORT_LEAD", 0.35))    # 미리 켜두는 거리 [m]

_state = {}


def start():
    stage = omni.usd.get_context().get_stage()
    switches = {}
    for name in ZONE:
        attr = stage.GetAttributeAtPath(SWITCH.format(sorter=name))
        if not attr:
            print(f"[sorter] 스위치 없음: {SWITCH.format(sorter=name)} "
                  f"— run_install_sorters.sh 를 먼저 돌려라")
            continue
        attr.Set(False)
        switches[name] = attr
    _state.update(stage=stage, switches=switches, routed={}, on=set())
    _state["physics_sub"] = get_physx_interface().subscribe_physics_step_events(_on_step)
    _state["timeline_sub"] = (
        omni.timeline.get_timeline_interface()
        .get_timeline_event_stream()
        .create_subscription_to_pop(_on_timeline)
    )
    print("[sorter] 색상 분류 시작 — 먼 곳부터 R/G/B (A43 팝업 휠)")
    for slug, (name, label) in ROUTE.items():
        where = "소터 OFF 통과" if name is None else f"{name}  y {ZONE[name][0]:.3f}..{ZONE[name][1]:.3f}"
        print(f"[sorter]   {slug}  {label:<16} {where}")


def stop():
    for attr in _state.get("switches", {}).values():
        attr.Set(False)
    _state["physics_sub"] = None
    _state["timeline_sub"] = None
    _state.clear()


def _on_timeline(event):
    if event.type == int(omni.timeline.TimelineEventType.STOP):
        for attr in _state.get("switches", {}).values():
            attr.Set(False)
        _state["routed"] = {}
        _state["on"] = set()


def _color_of(prim):
    """박스 색을 돌려준다. 카메라 인식이 붙으면 여기만 교체한다."""
    name = prim.GetName()
    for slug in ROUTE:
        if name.endswith(slug):
            return slug
    return None


def _pos(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return m.ExtractTranslation()


def _on_step(dt):
    stage = _state.get("stage")
    if stage is None:
        return
    root = stage.GetPrimAtPath(BOX_ROOT)
    if not root:
        return
    switches = _state["switches"]

    # 이번 스텝에 켜져 있어야 할 소터를 모은다.
    want = set()
    for prim in root.GetChildren():
        slug = _color_of(prim)
        if slug is None:
            continue
        name, label = ROUTE[slug]
        if name is None or name not in switches:
            continue
        y0, y1 = ZONE[name]
        y = _pos(prim)[1]
        if y0 - LEAD <= y <= y1:
            want.add(name)
            path = prim.GetPath().pathString
            if path not in _state["routed"]:
                _state["routed"][path] = label
                print(f"[sorter] {prim.GetName()} -> {label}  ({name} ON, y={y:.3f})")

    for name, attr in switches.items():
        on = name in want
        if on != (name in _state["on"]):
            attr.Set(on)
            if on:
                _state["on"].add(name)
            else:
                _state["on"].discard(name)
