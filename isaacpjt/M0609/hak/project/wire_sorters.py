#!/usr/bin/env python3
"""A43 소터의 롤러 레인에 컨베이어 구동 그래프를 붙인다.

에셋만 놓으면 롤러가 안 돈다. 다른 트랙과 같은 ConveyorBeltGraph
(OnTick -> ConveyorNode <- read_speed) 를 레인마다 만들어 줘야 한다.

그래프를 손으로 짜면 노드 타입·연결·변수까지 맞춰야 해서 틀리기 쉽다.
그래서 이미 도는 트랙(_10)의 그래프를 통째로 복사한 뒤, 안쪽 절대경로와
대상 프림만 새 이름으로 고쳐 쓴다. flatten_line.py 에서 A37->A06 조각을
복제할 때 쓴 방식과 같다.

레인
    Rollers      본선 (직진)
    Rollers_01   분기 (스퍼로 빠지는 쪽)

방향은 track_types.json 의 A43 정의를 따른다 (둘 다 [1,0,0]).
월드 방향은 배치 회전에 따라 달라지므로 적용 후 실측으로 확인해야 한다.

    ./run_wire_sorters.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
TEMPLATE = "/World/ConveyorTrack_10/ConveyorBeltGraph"

SORTERS = ["SorterBlue", "SorterGreen"]
# (그래프 이름, 대상 레인, direction)
LANES = [
    ("ConveyorBeltGraph", "Rollers", (1.0, 0.0, 0.0)),
    ("ConveyorBeltGraph_01", "Rollers_01", (1.0, 0.0, 0.0)),
]
VELOCITY = 0.5

import shutil
import time

from pxr import Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

if layer.GetPrimAtPath(Sdf.Path(TEMPLATE)) is None:
    raise SystemExit(f"템플릿 그래프가 없다: {TEMPLATE}")


def rewrite(prim_path, old, new):
    prim = layer.GetPrimAtPath(prim_path)
    if prim is None:
        return
    for prop in prim.properties:
        for listop in (getattr(prop, "connectionPathList", None),
                       getattr(prop, "targetPathList", None)):
            if listop is None:
                continue
            for field in ("explicitItems", "prependedItems", "appendedItems"):
                items = list(getattr(listop, field))
                if not items:
                    continue
                fixed = [Sdf.Path(str(p).replace(old, new, 1)) for p in items]
                if fixed != items:
                    setattr(listop, field, fixed)
    for child in prim.nameChildren:
        rewrite(prim_path.AppendChild(child.name), old, new)


for sorter in SORTERS:
    base = Sdf.Path(f"/World/{sorter}")
    if layer.GetPrimAtPath(base) is None:
        print(f"[없음] {sorter} — run_install_sorters.sh 를 먼저 돌려라")
        continue
    print(f"[{sorter}]")
    for gname, lane, direction in LANES:
        dst = base.AppendChild(gname)
        if layer.GetPrimAtPath(dst) is not None:
            del layer.GetPrimAtPath(base).nameChildren[gname]
        Sdf.CopySpec(layer, Sdf.Path(TEMPLATE), layer, dst)
        rewrite(dst, "/World/ConveyorTrack_10/ConveyorBeltGraph",
                f"/World/{sorter}/{gname}")
        rewrite(dst, "/World/ConveyorTrack_10/", f"/World/{sorter}/")

        node = dst.AppendChild("ConveyorNode")
        rel = layer.GetRelationshipAtPath(node.AppendProperty("inputs:conveyorPrim"))
        rel.targetPathList.explicitItems = [base.AppendChild(lane)]

        d = layer.GetAttributeAtPath(node.AppendProperty("inputs:direction"))
        d.default = Gf.Vec3f(*direction)

        v = layer.GetAttributeAtPath(dst.AppendProperty("graph:variable:Velocity"))
        v.default = VELOCITY

        print(f"  {gname:<22} -> {lane:<12} dir={direction}  Velocity={VELOCITY}")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
