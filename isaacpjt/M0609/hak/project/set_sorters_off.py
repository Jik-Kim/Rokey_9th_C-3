#!/usr/bin/env python3
"""세 소터의 팝업 휠을 끈 상태로 씬에 박아둔다.

분기 롤러(Rollers_01)가 아직 아무 벨트에도 안 닿아 있다. 분류를 켜면
팝업 휠이 박스를 허공으로 밀어내서 튕기고 막힌다. 분기에 받는 벨트를
놓기 전까지는 꺼진 상태가 기본이어야 한다.

SorterRed 는 이 레이어에 스펙이 없어 에셋 기본값을 쓰고 있었다.
의도를 파일에 명시해두려고 셋 다 오버라이드를 만든다.

    ./run_set_sorters_off.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

import shutil
import time

from pxr import Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)
sorters = [c.name for c in layer.GetPrimAtPath("/World").nameChildren
           if layer.GetPrimAtPath(Sdf.Path(f"/World/{c.name}/Sorter/ActionGraph"))]

for name in sorters:
    path = Sdf.Path(f"/World/{name}/Sorter/ActionGraph/binary_switch")
    prim = layer.GetPrimAtPath(path)
    if prim is None:
        prim = Sdf.CreatePrimInLayer(layer, path)
        prim.specifier = Sdf.SpecifierOver
    a = layer.GetAttributeAtPath(path.AppendProperty("inputs:value"))
    if a is None:
        a = Sdf.AttributeSpec(prim, "inputs:value", Sdf.ValueTypeNames.Bool)
        print(f"  [오버라이드 생성] {name}")
    old = a.default
    a.default = False
    print(f"  {name}/binary_switch  {old} -> False")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
