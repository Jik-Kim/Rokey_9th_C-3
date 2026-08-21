#!/usr/bin/env python3
"""Isaac Sim 을 안 띄우고 test1.usd 의 모든 ConveyorBeltGraph 에 동일 Velocity 를 박는다.

Script Editor 용은 set_conveyor.py (열려 있는 스테이지를 고침, Ctrl+S 필요).
이쪽은 파일을 직접 고치므로 저장 누락이 없다.

    ./run_set_conveyor.sh

Sdf 레이어만 열기 때문에 원격 컨베이어 에셋을 안 받아온다 (~1초).
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
VELOCITY = 0.5

from pxr import Sdf

layer = Sdf.Layer.FindOrOpen(USD_PATH)
if layer is None:
    raise SystemExit(f"레이어를 못 열었다: {USD_PATH}")

count = 0
changed = 0


def visit(path):
    global count, changed
    spec = layer.GetPrimAtPath(path)
    if spec is None or spec.typeName != "OmniGraph":
        return
    attr = layer.GetAttributeAtPath(path.AppendProperty("graph:variable:Velocity"))
    if not attr:
        return
    old = attr.default
    count += 1
    if old != VELOCITY:
        attr.default = VELOCITY
        changed += 1
        print(f"  {old} -> {VELOCITY}   {path}")


layer.Traverse(Sdf.Path("/"), visit)
layer.Save()

print(f"\nConveyorBeltGraph {count}개 중 {changed}개 변경, 전부 Velocity={VELOCITY}")
print(f"저장 완료: {USD_PATH}")
