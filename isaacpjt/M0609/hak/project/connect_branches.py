#!/usr/bin/env python3
"""분기 출구와 받는 벨트를 잇는다.

실측 연결 성분 (Isaac Sim 런타임):
    [본선] ConveyorTrack -> _17(커브) -> _03 -> SorterBlue -> SorterGreen
           -> SorterRed -> _01                     7개 면, 끊긴 곳 없음
    [섬 1] ConveyorTrack_10 + _04                  본선과 분리
    [섬 2] SorterBlue/Rollers_01                   갈 곳 없음
    [섬 3] SorterGreen/Rollers_01                  갈 곳 없음
    [섬 4] SorterRed/Rollers_01                    갈 곳 없음

분기 3개가 전부 허공에서 끝난다. 그래서 분기로 빠진 박스가 1.24 m 아래
프레임(z=0.780)으로 떨어져 대각선으로 쌓였다.

배정 (먼 곳부터 RGB)
    빨강 3호  본선 끝 _01        소터 통과. 분기 필요 없음.
    초록 4호  SorterGreen 분기 -> _10 -> _04
    파랑 5호  SorterBlue  분기 -> (받는 벨트 없음. 새로 놓는다)

이 스크립트는 [1] 초록 분기와 _10 사이 1.242 m 를 없애고
[2] 파랑 분기용 받는 벨트를 새로 놓는다.

    ./run_connect_branches.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
A05 = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
       "/Assets/Isaac/5.1/Isaac/Props/Conveyors/ConveyorBelt_A05.usd")

# 실측값
# 상대 이동으로 짜면 다시 돌릴 때마다 또 밀린다. 절대 목표값으로 못박는다.
#   원래   _10 x -2.2231 / _04 x -2.2727
#   간격   SorterGreen/Rollers_01 끝 -7.332  ->  _10 시작 -6.091  = 1.242 m
#   겹침   0.05
GREEN_GAP = 1.242
TARGET_X = {"ConveyorTrack_10": -3.5151, "ConveyorTrack_04": -3.5647}

BELT_Z = 0.1307            # A05 원점 (벨트면 0.900)

# 파랑 분기 출구 (SorterBlue/Rollers_01: x -8.988..-7.332, y 4.848..7.084, 45도)
BLUE_SPUR = dict(
    name="ConveyorTrack_Blue",
    # 분기 출구 바로 뒤에서 시작해 같은 45도 방향으로 뻗는다
    translate=(-7.300, 5.966, BELT_Z),
    yaw=45.0,
    # 3.0 m 로 뽑았더니 끝이 _10 (y 8.386) 에 닿아 초록 라인과 합쳐졌다.
    # 색을 갈라놓으려면 독립이어야 한다. 2.0 m 로 자른다.
    scale_x=1.0,
)

import math
import shutil
import time

from pxr import Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

# ---- [1] 초록 분기 <-> _10 간격 제거 ----
print(f"[1] 초록 분기와 받는 벨트 잇기  (간격 {GREEN_GAP:.3f} m)")
for name, target in TARGET_X.items():
    a = layer.GetAttributeAtPath(Sdf.Path(f"/World/{name}").AppendProperty("xformOp:translate"))
    if a is None:
        print(f"    {name}: translate 없음 — 건너뜀")
        continue
    o = Gf.Vec3d(a.default)
    a.default = Gf.Vec3d(target, o[1], o[2])
    print(f"    {name}: x {o[0]:+.4f} -> {target:+.4f}")

# ---- [2] 파랑 분기용 받는 벨트 ----
print(f"\n[2] 파랑 분기용 받는 벨트 신설")
path = Sdf.Path(f"/World/{BLUE_SPUR['name']}")
if layer.GetPrimAtPath(path):
    layer.RemovePrim(path)
    print(f"    기존 {BLUE_SPUR['name']} 제거")

prim = Sdf.CreatePrimInLayer(layer, path)
prim.specifier = Sdf.SpecifierDef
prim.typeName = "Xform"
refs = Sdf.ReferenceListOp()
refs.prependedItems = [Sdf.Reference(A05)]
prim.SetInfo("references", refs)

half = math.radians(BLUE_SPUR["yaw"]) / 2.0
for pname, ptype, val in (
        ("xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*BLUE_SPUR["translate"])),
        ("xformOp:orient", Sdf.ValueTypeNames.Quatd,
         Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half)))),
        ("xformOp:scale", Sdf.ValueTypeNames.Float3,
         Gf.Vec3f(BLUE_SPUR["scale_x"], 1.0, 1.0)),
):
    Sdf.AttributeSpec(prim, pname, ptype).default = val
Sdf.AttributeSpec(prim, "xformOpOrder", Sdf.ValueTypeNames.TokenArray).default = [
    "xformOp:translate", "xformOp:orient", "xformOp:scale"]
print(f"    {BLUE_SPUR['name']}  위치 {BLUE_SPUR['translate']}  yaw {BLUE_SPUR['yaw']}도  "
      f"x배율 {BLUE_SPUR['scale_x']}")

# ---- [3] 새 벨트의 컨베이어 그래프 — 기존 것을 복제한다 ----
SRC = "/World/ConveyorTrack_01/ConveyorBeltGraph"
print(f"\n[3] 컨베이어 그래프 복제  {SRC}  ->  {path}/ConveyorBeltGraph")
dst = path.AppendChild("ConveyorBeltGraph")
Sdf.CopySpec(layer, Sdf.Path(SRC), layer, dst)


def rewrite(p):
    """복제한 그래프 안의 /World/ConveyorTrack_01/... 경로를 새 프림으로 돌린다."""
    spec = layer.GetPrimAtPath(p)
    if spec is None:
        return
    for prop in spec.properties:
        for lst in (getattr(prop, "connectionPathList", None),
                    getattr(prop, "targetPathList", None)):
            if lst is None:
                continue
            for slot in ("explicitItems", "prependedItems", "appendedItems", "addedItems"):
                items = list(getattr(lst, slot, []))
                if not items:
                    continue
                new = [Sdf.Path(t.pathString.replace("/World/ConveyorTrack_01/",
                                                     f"{path.pathString}/"))
                       for t in items]
                if new != items:
                    del getattr(lst, slot)[:]
                    for t in new:
                        getattr(lst, slot).append(t)
    for ch in spec.nameChildren:
        rewrite(p.AppendChild(ch.name))


rewrite(dst)
vel = layer.GetAttributeAtPath(dst.AppendProperty("graph:variable:Velocity"))
if vel is not None:
    vel.default = 0.5
    print(f"    Velocity = 0.5")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
