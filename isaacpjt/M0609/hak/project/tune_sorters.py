#!/usr/bin/env python3
"""A43 소터 팝업 휠의 방향·속도를 조정한다.

에셋 기본값은 Direction = -45.0, SorterSpeed = -3.0 인데, 이 씬의 배치
(+90 deg 회전, 분기가 월드 +X)에서는 휠이 반대인 -X 로 민다. 검증에서
12개 중 10개가 본선 서쪽(x -10.2..-10.6)으로 떨어졌다.

Direction 부호를 뒤집으면 미는 쪽이 바뀐다.

    ./run_tune_sorters.sh                    # 기본: Direction=+45
    PL_DIR=-45 PL_SPEED=-3 ./run_tune_sorters.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
# 소터 목록을 손으로 적으면 새로 추가한 조각이 빠진다. 실제로 SorterRed 를
# 나중에 넣었을 때 이 목록에 없어서 에셋 기본값(SorterSpeed=-3.0)이 남았고,
# 그 데크가 역방향 3 m/s 로 돌아 박스가 세 번째 소터에서 전부 멈췄다.
# 그래서 Sorter/ActionGraph 를 가진 프림을 스테이지에서 찾아 쓴다.
SORTERS = None      # None 이면 자동 탐색

import os
import shutil
import time

from pxr import Sdf

DIRECTION = float(os.environ.get("PL_DIR", 45.0))
# 소터 데크 속도. 에셋 기본 -3.0 은 음수라, 분기를 안 켠 상태(회전각 0)에서
# 데크가 역방향으로 3 m/s 로 돈다. 라인 속도 0.5 의 6배로 뒤로 밀어내서
# 박스가 소터 입구에 쌓인다. 양수 0.5 면 OFF 일 때 그대로 통과하고,
# ON 이면 Direction(+45도) 만큼 회전한 방향으로 분기로 밀어낸다.
SPEED = os.environ.get("PL_SPEED", "0.5")

# 소터 데크(팝업 휠) 자체의 이송 속도.
# 에셋 기본값이 0.0 이라 분기를 안 켠 박스가 소터 구역에서 그대로 멈춘다.
# 검증에서 박스가 소터 앞(y 4.0~5.3)에 줄줄이 섰던 원인이다.
# 본선과 같은 0.5 를 주면 OFF 일 때 그냥 통과한다.
SORTER_VEL = float(os.environ.get("PL_SORTER_VEL", 0.5))

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

if SORTERS is None:
    found = []
    def _find(path):
        spec = layer.GetPrimAtPath(path)
        if spec is None or path.pathString.count("/") != 2:
            return
        if layer.GetPrimAtPath(path.AppendChild("Sorter").AppendChild("ActionGraph")):
            found.append(path.name)
    layer.Traverse(Sdf.Path("/"), _find)
    SORTERS = sorted(found)
    print(f"소터 자동 탐색: {SORTERS}\n")

for name in SORTERS:
    base = f"/World/{name}/Sorter/ActionGraph"
    for node, value in (("Direction", DIRECTION),
                        ("SorterSpeed", float(SPEED) if SPEED else None),
                        # conveyor_belt.inputs:velocity 는 SorterSpeed 에
                        # 연결돼 있어 기본값을 써도 무시된다. 건드리지 않는다.
                        ):
        if value is None:
            continue
        prop = "inputs:velocity" if node == "conveyor_belt" else "inputs:value"
        path = Sdf.Path(f"{base}/{node}").AppendProperty(prop)
        a = layer.GetAttributeAtPath(path)
        if a is None:
            # 에셋 기본값이라 이 레이어에 스펙이 없다. 오버라이드를 만든다.
            prim = layer.GetPrimAtPath(Sdf.Path(f"{base}/{node}"))
            if prim is None:
                prim = Sdf.CreatePrimInLayer(layer, Sdf.Path(f"{base}/{node}"))
                prim.specifier = Sdf.SpecifierOver
            a = Sdf.AttributeSpec(prim, prop, Sdf.ValueTypeNames.Float)
            print(f"  [오버라이드 생성] {path}")
        old = a.default
        a.default = value
        print(f"  {name}/{node:<12} {old} -> {value}")

# 비어 있는 컨베이어 Velocity 를 채운다. 조각을 새로 놓으면 자주 빠진다.
print("\n[Velocity 비어 있는 그래프]")
filled = 0
def _fill(path):
    global filled
    spec = layer.GetPrimAtPath(path)
    if not (spec and spec.typeName == "Xform" and path.pathString.count("/") == 2):
        return
    for child in spec.nameChildren:
        if child.typeName != "OmniGraph":
            continue
        a = layer.GetAttributeAtPath(
            path.AppendChild(child.name).AppendProperty("graph:variable:Velocity"))
        if a is not None and not a.default:
            a.default = 0.5
            filled += 1
            print(f"  {path.name}/{child.name}  -> 0.5")
layer.Traverse(Sdf.Path("/"), _fill)
if not filled:
    print("  없음")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
