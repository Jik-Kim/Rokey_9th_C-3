#!/usr/bin/env python3
"""소터 그래프 배선을 고친다.

1) 프림을 복사해서 소터를 늘리면 ConveyorNode.inputs:conveyorPrim 의
   타깃 경로가 원본 소터를 그대로 가리킨다. 실제로 SorterRed 의 두 그래프가
   /World/SorterGreen/Rollers 를 몰고 있어서 SorterRed 데크는 아무도 구동하지
   않았다. 타깃을 자기 자신으로 되돌린다.

2) 분기 롤러(Rollers_01)는 로컬 +X 가 월드 45도라, 기본 direction (1,0,0)
   이면 팝업 휠이 꺼져 있어도 항상 북동쪽으로 민다. 검증에서 10개 중 2개가
   이렇게 이탈했다.
   그렇다고 속도를 0 으로 세우면 이번엔 죽은 마찰면이 돼서 10개 전부 그
   자리에 섰다. 박스는 이 롤러 위를 지날 수밖에 없는 구조다.
   그래서 세우지 않고 방향만 돌린다. 로컬 (0.7071, 0.7071, 0) 이 월드 +Y 다.
   분류할 때만 sorter.py 가 (1,0,0) 으로 되돌려 45도로 배출한다.

    ./run_fix_sorter_wiring.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
BRANCH_GRAPH = "ConveyorBeltGraph_01"   # Rollers_01 (분기) 을 모는 그래프
BRANCH_VEL = 0.5
# 로컬 +X 가 월드 +45도인 프레임에서 월드 +Y 를 얻는 로컬 방향.
#   local = R(-45) * (0,1) = (0.70710678, 0.70710678)
BRANCH_STRAIGHT = (0.70710678, 0.70710678, 0.0)

import shutil
import time

from pxr import Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

sorters = []
def _find(path):
    if path.pathString.count("/") != 2:
        return
    if layer.GetPrimAtPath(path.AppendChild("Sorter").AppendChild("ActionGraph")):
        sorters.append(path.name)
layer.Traverse(Sdf.Path("/"), _find)
sorters.sort()
print(f"소터: {sorters}\n")

print("[1] conveyorPrim 타깃 교정")
for name in sorters:
    root = f"/World/{name}"
    for graph in ("ConveyorBeltGraph", BRANCH_GRAPH):
        gp = layer.GetPrimAtPath(f"{root}/{graph}")
        if gp is None:
            continue
        for node in gp.nameChildren:
            rel = layer.GetPropertyAtPath(
                Sdf.Path(f"{root}/{graph}/{node.name}").AppendProperty("inputs:conveyorPrim"))
            if rel is None:
                continue
            items = list(rel.targetPathList.GetAddedOrExplicitItems())
            fixed = []
            for t in items:
                # /World/<다른소터>/Rollers... -> /World/<이소터>/Rollers...
                parts = t.pathString.split("/")
                if len(parts) > 3 and parts[2] in sorters and parts[2] != name:
                    parts[2] = name
                    fixed.append(Sdf.Path("/".join(parts)))
                else:
                    fixed.append(t)
            if fixed != items:
                rel.targetPathList.ClearEditsAndMakeExplicit()
                for t in fixed:
                    rel.targetPathList.explicitItems.append(t)
                print(f"  {name}/{graph}: {items[0]} -> {fixed[0]}")
            else:
                print(f"  {name}/{graph}: OK ({items[0] if items else '없음'})")

print(f"\n[2] 분기 롤러 평소 직진 (direction -> {BRANCH_STRAIGHT}, Velocity -> {BRANCH_VEL})")
for name in sorters:
    a = layer.GetAttributeAtPath(
        Sdf.Path(f"/World/{name}/{BRANCH_GRAPH}").AppendProperty("graph:variable:Velocity"))
    if a is None:
        print(f"  {name}: Velocity 스펙 없음")
    else:
        old = a.default
        a.default = BRANCH_VEL
        print(f"  {name}: Velocity {old} -> {BRANCH_VEL}")

    gp = layer.GetPrimAtPath(f"/World/{name}/{BRANCH_GRAPH}")
    if gp is None:
        continue
    for node in gp.nameChildren:
        d = layer.GetAttributeAtPath(
            Sdf.Path(f"/World/{name}/{BRANCH_GRAPH}/{node.name}").AppendProperty("inputs:direction"))
        if d is None:
            continue
        old = d.default
        d.default = Gf.Vec3f(*BRANCH_STRAIGHT)
        print(f"  {name}: {node.name}.direction {old} -> {d.default}")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
