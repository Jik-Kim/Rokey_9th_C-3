#!/usr/bin/env python3
"""키 큰 컨베이어 조각을 같은 규격의 낮은 계열로 교체한다.

문제
    A06 / A03 은 원래 벨트면이 1780 mm 다. 이걸 900 mm 로 쓰려고 z 를
    -0.881 로 내려놨더니 다리와 프레임이 지하 879 mm 에 묻혔다.
    반대로 A05 / A43 은 원래 770 mm 라 +130 mm 띄워야 900 이 된다.
    한 라인에 파묻힌 조각과 떠 있는 조각이 섞여 있었다.

track_types.json 의 start_level 이 계열을 가른다.
    start_level 1 = BELT   계열, 벨트면 1780
    start_level 0 = ROLLER 계열, 벨트면  770

같은 형상의 짝이 있다.
    A06 (STRAIGHT, angle NONE)        -> A05 (STRAIGHT, angle NONE)
    A03 (STRAIGHT, angle HALF/SMALL)  -> A02 (STRAIGHT, angle HALF/SMALL)

교체하면 전 조각이 +130 mm 만 뜨므로 낮은 받침대로 가릴 수 있다.

주의: 하위 프림 이름이 Belt -> Rollers 로 바뀐다. 그래프의
inputs:conveyorPrim 을 같이 안 고치면 벨트가 통째로 죽는다.

    ./run_swap_to_low.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
BASE = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
        "/Assets/Isaac/5.1/Isaac/Props/Conveyors/")

SWAP = {"ConveyorBelt_A06": "ConveyorBelt_A05",
        "ConveyorBelt_A03": "ConveyorBelt_A02"}

# A05 계열의 벨트면 실측 상면 (에셋 로컬). ConveyorTrack_04 / _01 이 쓰는 값과 같다.
LOW_TOP = 0.7693
TARGET_TOP = 0.900
NEW_Z = round(TARGET_TOP - LOW_TOP, 4)

# 교체 후 존재하지 않게 되는 하위 프림 오버라이드
STALE_PREFIX = ("SM_ConveyorBelt_A06", "SM_ConveyorBelt_A03", "SM_ConveyorBelt_A37")

import shutil
import time

from pxr import Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")
print(f"새 원점 z = {TARGET_TOP} - {LOW_TOP} = {NEW_Z}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

targets = []
for child in layer.GetPrimAtPath("/World").nameChildren:
    prim = layer.GetPrimAtPath(f"/World/{child.name}")
    if not prim.HasInfo("references"):
        continue
    refs = prim.GetInfo("references")
    for slot in ("prependedItems", "explicitItems", "appendedItems", "addedItems"):
        for r in getattr(refs, slot, []):
            asset = r.assetPath.split("/")[-1].replace(".usd", "")
            if asset in SWAP:
                targets.append((child.name, slot, asset, SWAP[asset]))

if not targets:
    print("교체할 조각이 없다 (이미 낮은 계열인가?)")
    raise SystemExit(0)

for name, slot, old, new in targets:
    print(f"=== {name}:  {old} -> {new} ===")
    path = Sdf.Path(f"/World/{name}")
    prim = layer.GetPrimAtPath(path)

    # 1) 참조 교체
    refs = prim.GetInfo("references")
    items = [Sdf.Reference(BASE + new + ".usd") if
             r.assetPath.split("/")[-1].replace(".usd", "") == old else r
             for r in getattr(refs, slot, [])]
    new_refs = Sdf.ReferenceListOp()
    setattr(new_refs, slot, items)
    prim.SetInfo("references", new_refs)
    print(f"  참조 교체")

    # 2) 원점 z
    a = layer.GetAttributeAtPath(path.AppendProperty("xformOp:translate"))
    if a is not None:
        o = Gf.Vec3d(a.default)
        a.default = Gf.Vec3d(o[0], o[1], NEW_Z)
        print(f"  z {o[2]:+.4f} -> {NEW_Z:+.4f}   (지면 아래 {-o[2]*1000:.0f}mm -> 위 {NEW_Z*1000:.0f}mm)")

    # 3) Belt -> Rollers  (그래프 타깃)
    for g in prim.nameChildren:
        if g.typeName != "OmniGraph":
            continue
        for node in layer.GetPrimAtPath(path.AppendChild(g.name)).nameChildren:
            rel = layer.GetPropertyAtPath(
                path.AppendChild(g.name).AppendChild(node.name)
                    .AppendProperty("inputs:conveyorPrim"))
            if rel is None:
                continue
            items2 = list(rel.targetPathList.GetAddedOrExplicitItems())
            fixed = [Sdf.Path(t.pathString.replace("/Belt", "/Rollers")) for t in items2]
            if fixed != items2:
                rel.targetPathList.ClearEditsAndMakeExplicit()
                for t in fixed:
                    rel.targetPathList.explicitItems.append(t)
                print(f"  그래프 타깃  {items2[0]} -> {fixed[0]}")

    # 4) 사라진 하위 프림 오버라이드 제거
    for child in list(prim.nameChildren):
        cn = child.name
        if cn == "Belt" or cn.startswith(STALE_PREFIX):
            del layer.GetPrimAtPath(path).nameChildren[cn]
            print(f"  오버라이드 제거  {cn}")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
print("\n교체 후 실제 벨트면 높이는 런타임에서 재확인해야 한다 "
      "(A02 커브의 상면이 A05 와 다를 수 있다).")
