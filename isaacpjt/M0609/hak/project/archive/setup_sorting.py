#!/usr/bin/env python3
"""색상 분류를 위한 씬 준비.

분기 구조는 이미 맞물려 있다.
    _05/Belt     x -9.95..-9.05            본선 +Y 레인
    _05/Belt_01  x -9.06..-7.05  y 4.96..5.86   분기 레인 (가로)
    _12/Belt     x -7.32..-2.29  y 4.97..5.87   스퍼 (0.28m 물림)
    _06/Belt_01  x -9.06..-7.05  y 8.75..9.65
    _10/Belt     x -7.22..-2.26  y 8.76..9.66

고칠 것
  1. Belt_01 이 월드 -X 로 민다. 스퍼는 +X 쪽이라 반대다 -> direction 반전.
     A24 안에서 Belt_01 이 자체 -90 deg 회전을 갖고 있어서 값만 보면 헷갈린다.
     실제 월드 방향은 bbox 로 확인했다.
  2. _04 (스퍼 끝 롤러) Velocity 가 비어 있어 멈춰 있다 -> 0.5.

본선 레인(x -9.5)과 Belt_01(x >= -9.06)은 서로 안 겹친다. 5호(480mm)를 놓아도
박스 오른쪽 끝이 -9.256 이라 Belt_01 에 안 닿는다. 그래서 벨트만으로는 분기가
일어나지 않고, 색을 보고 옆으로 밀어주는 주체가 필요하다 -> sorter.py.

    ./run_setup_sorting.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

DIVERT_LANES = ["ConveyorTrack_05", "ConveyorTrack_06"]
VELOCITY = 0.5

import shutil
import time

from pxr import Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

print("[1] 분기 레인 Belt_01 방향 반전 (월드 -X -> +X)")
for name in DIVERT_LANES:
    node = Sdf.Path(f"/World/{name}/ConveyorBeltGraph_01/ConveyorNode")
    d = layer.GetAttributeAtPath(node.AppendProperty("inputs:direction"))
    if d is None or d.default is None:
        print(f"  [없음] {name}/ConveyorBeltGraph_01")
        continue
    v = d.default
    d.default = type(v)(-v[0], -v[1], -v[2])
    print(f"  {name}/Belt_01  {tuple(v)} -> {tuple(d.default)}")

print("\n[2] 멈춰 있는 그래프 Velocity 채우기")
def visit(p):
    s = layer.GetPrimAtPath(p)
    if not (s and s.typeName == "Xform" and p.pathString.count("/") == 2
            and "ConveyorTrack" in p.name):
        return
    for child in s.nameChildren:
        if child.typeName != "OmniGraph":
            continue
        a = layer.GetAttributeAtPath(
            p.AppendChild(child.name).AppendProperty("graph:variable:Velocity"))
        if a is not None and not a.default:
            a.default = VELOCITY
            print(f"  {p.name}/{child.name}  -> {VELOCITY}")
layer.Traverse(Sdf.Path("/"), visit)

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
