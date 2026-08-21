#!/usr/bin/env python3
"""ConveyorNode 의 끊긴 입력 배선을 복구한다.

증상
  파란 스퍼(_12) 위에서 박스가 x -5.3 부근에 줄줄이 멈춘다.
  런타임에 physxSurfaceVelocity 를 찍어보면 _12/Belt 만 (0,0,0) 이다.
      _05/Belt_01  (-0.5, 0, 0)
      _12/Belt     ( 0,   0, 0)   <- 이것
      _04/Rollers  ( 0.5, 0, 0)
      _10/Belt     (-0.5, 0, 0)

원인
  _12 의 ConveyorNode 에 inputs:velocity 속성이 아예 없다. 정상인 _10 은
  read_speed.outputs:value 로 연결돼 있다. 입력이 없으니 0 이 들어가고
  표면속도가 0 이 된다. graph:variable:Velocity 는 0.5 로 멀쩡해서
  스테이지만 봐서는 정상으로 보인다.

  조각을 복제하거나 에셋을 갈아끼우는 과정에서 연결이 빠진 것으로 보인다.

    ./run_fix_graph_wiring.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

# (속성 이름, 타입, 연결할 소스 노드.출력)
WIRING = [
    ("inputs:velocity", "Float", "read_speed.outputs:value"),
    ("inputs:delta", "Float", "OnTick.outputs:deltaSeconds"),
    ("inputs:onStep", "UInt", "OnTick.outputs:tick"),
]

import shutil
import time

from pxr import Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

fixed = 0
checked = 0


def visit(p):
    global fixed, checked
    spec = layer.GetPrimAtPath(p)
    if not (spec and spec.typeName == "Xform"
            and p.pathString.count("/") == 2 and "ConveyorTrack" in p.name):
        return
    for child in spec.nameChildren:
        if child.typeName != "OmniGraph":
            continue
        graph = p.AppendChild(child.name)
        node = layer.GetPrimAtPath(graph.AppendChild("ConveyorNode"))
        if node is None:
            continue
        checked += 1
        for name, tname, source in WIRING:
            src = graph.AppendChild(source.split(".")[0]).AppendProperty(
                source.split(".", 1)[1])
            prop = node.properties.get(name)
            if prop is None:
                prop = Sdf.AttributeSpec(node, name,
                                         getattr(Sdf.ValueTypeNames, tname))
                prop.custom = True
                print(f"  [속성 신규] {p.name}/{child.name}.{name}")
            have = (list(prop.connectionPathList.explicitItems)
                    + list(prop.connectionPathList.prependedItems))
            if any(str(x) == str(src) for x in have):
                continue
            prop.connectionPathList.prependedItems = [src]
            fixed += 1
            print(f"  [연결] {p.name}/{child.name}.{name} -> {source}")


layer.Traverse(Sdf.Path("/"), visit)
layer.Save()

print(f"\nConveyorNode {checked}개 점검, 배선 {fixed}건 복구")
print(f"저장 완료: {USD_PATH}")
