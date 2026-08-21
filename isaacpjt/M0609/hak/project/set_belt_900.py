#!/usr/bin/env python3
"""라인 전 구간의 접촉면(벨트/롤러 상면)을 지면 위 900 mm 로 맞춘다.

왜 이렇게 하나
    A06 / A03 은 벨트면이 원래 1780 mm 다. 이걸 900 으로 쓰려고 조각을
    z=-0.881 로 내렸더니 다리와 프레임이 지하 881 mm 에 묻혔다. 반대로
    A05 / A43 을 +131 mm 띄우면 다리가 공중에 뜬다. 한 라인에 묻힌 조각과
    뜬 조각이 섞여서 보기 싫었다.

    그래서 두 가지를 같이 한다.
      1) A06 -> A05, A03 -> A02 로 낮은 계열 교체 (같은 형상의 짝).
      2) 조각 원점 z 는 0 (다리를 땅에 붙이고), z 스케일로 상면을 900 에 맞춘다.
    묻히지도 뜨지도 않는다.

    A05/A43 은 이미 z 스케일 2.0~2.26 으로 늘려 1780 에 맞춰 놓은 상태였다.
    이 스크립트가 그 스케일을 1.15~1.17 로 되돌린다.

주의: 교체하면 하위 프림 이름이 Belt -> Rollers 로 바뀐다. 그래프의
inputs:conveyorPrim 을 같이 안 고치면 그 조각이 통째로 죽는다.

두 번 돌린다
    1회차: NATIVE_TOP 이 비어 있으면 z 스케일을 1.0 으로 두고 저장한다.
           그 상태를 헤드리스로 재서 조각별 고유 상면을 얻는다.
    2회차: 그 값을 NATIVE_TOP 에 채우고 다시 돌리면 스케일이 잡힌다.
    교체는 멱등이다 (이미 A05 면 건너뛴다).

    ./run_set_belt_900.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
BASE = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
        "/Assets/Isaac/5.1/Isaac/Props/Conveyors/")

TARGET_TOP = 0.900

SWAP = {"ConveyorBelt_A06": "ConveyorBelt_A05",
        "ConveyorBelt_A03": "ConveyorBelt_A02"}

# 교체 후 존재하지 않게 되는 하위 프림 오버라이드
STALE_PREFIX = ("SM_ConveyorBelt_A06", "SM_ConveyorBelt_A03", "SM_ConveyorBelt_A37")

# 조각별 고유 상면 [m] — z 스케일 1.0, 원점 z=0 일 때의 실측값.
# 비워 두면 스케일을 1.0 으로 두고 저장한다 (측정용 1회차).
NATIVE_TOP = {}

import json
import os
import shutil
import time

from pxr import Gf, Sdf

_nt = os.environ.get("PL_NATIVE")
if _nt:
    NATIVE_TOP = json.loads(_nt)

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)
world = layer.GetPrimAtPath("/World")


def asset_of(prim):
    """이 조각이 참조하는 컨베이어 에셋 이름과 슬롯."""
    if not prim.HasInfo("references"):
        return None, None
    refs = prim.GetInfo("references")
    for slot in ("prependedItems", "explicitItems", "appendedItems", "addedItems"):
        for r in getattr(refs, slot, []):
            name = r.assetPath.split("/")[-1].replace(".usd", "")
            if name.startswith("ConveyorBelt_"):
                return name, slot
    return None, None


def swap_ref(layer, path, prim, slot, old, new):
    refs = prim.GetInfo("references")
    items = [Sdf.Reference(BASE + new + ".usd")
             if r.assetPath.split("/")[-1].replace(".usd", "") == old else r
             for r in getattr(refs, slot, [])]
    op = Sdf.ReferenceListOp()
    setattr(op, slot, items)
    prim.SetInfo("references", op)

    # 그래프 타깃 Belt -> Rollers
    for g in prim.nameChildren:
        if g.typeName != "OmniGraph":
            continue
        for node in layer.GetPrimAtPath(path.AppendChild(g.name)).nameChildren:
            rel = layer.GetPropertyAtPath(
                path.AppendChild(g.name).AppendChild(node.name)
                    .AppendProperty("inputs:conveyorPrim"))
            if rel is None:
                continue
            cur = list(rel.targetPathList.GetAddedOrExplicitItems())
            fixed = [Sdf.Path(t.pathString.replace("/Belt", "/Rollers")) for t in cur]
            if fixed != cur:
                rel.targetPathList.ClearEditsAndMakeExplicit()
                for t in fixed:
                    rel.targetPathList.explicitItems.append(t)
                print(f"    그래프 타깃 {cur[0]} -> {fixed[0]}")

    # 사라지는 하위 오버라이드 제거
    stale = [c.name for c in prim.nameChildren
             if c.name == "Belt" or c.name.startswith(STALE_PREFIX)]
    for name in stale:
        del prim.nameChildren[name]
        print(f"    오버라이드 제거 {name}")

    # Rollers 오버라이드를 만들고 표면속도를 켠다.
    # 이걸 안 해 주면 조각은 살아 있는데 박스가 안 밀린다.
    roll = layer.GetPrimAtPath(path.AppendChild("Rollers"))
    if roll is None:
        roll = Sdf.PrimSpec(prim, "Rollers", Sdf.SpecifierOver)
    if roll.properties.get("physxSurfaceVelocity:surfaceVelocityEnabled") is None:
        a = Sdf.AttributeSpec(roll, "physxSurfaceVelocity:surfaceVelocityEnabled",
                              Sdf.ValueTypeNames.Bool)
        a.default = True
        print("    Rollers 표면속도 ON")


rows = []
for child in list(world.nameChildren):
    path = Sdf.Path(f"/World/{child.name}")
    prim = layer.GetPrimAtPath(path)
    asset, slot = asset_of(prim)
    if asset is None:
        continue

    print(f"=== {child.name}  ({asset})")
    if asset in SWAP:
        print(f"    참조 교체 {asset} -> {SWAP[asset]}")
        swap_ref(layer, path, prim, slot, asset, SWAP[asset])
        asset = SWAP[asset]

    t = layer.GetAttributeAtPath(path.AppendProperty("xformOp:translate"))
    s = layer.GetAttributeAtPath(path.AppendProperty("xformOp:scale"))
    old_z = float(Gf.Vec3d(t.default)[2]) if t is not None else 0.0
    old_sz = float(Gf.Vec3d(s.default)[2]) if s is not None else 1.0

    native = NATIVE_TOP.get(child.name)
    new_sz = round(TARGET_TOP / native, 6) if native else 1.0

    if t is not None:
        o = Gf.Vec3d(t.default)
        t.default = Gf.Vec3d(o[0], o[1], 0.0)
    if s is not None:
        o = Gf.Vec3d(s.default)
        s.default = Gf.Vec3d(o[0], o[1], new_sz)

    note = f"고유상면 {native * 1000:.1f}mm" if native else "측정 1회차 (스케일 1.0)"
    print(f"    z {old_z:+.4f} -> 0.0000   scale.z {old_sz:.4f} -> {new_sz:.4f}   {note}")
    rows.append(child.name)

layer.Save()
print(f"\n저장 완료: {USD_PATH}  (조각 {len(rows)}개)")
if not NATIVE_TOP:
    print("측정 1회차다. 헤드리스로 상면을 재서 PL_NATIVE 로 다시 돌려라.")
