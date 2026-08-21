#!/usr/bin/env python3
"""한 컨베이어 조각을 두 개 이상의 IsaacConveyor 노드가 미는 것을 없앤다.

증상
    박스가 특정 조각에 올라타는 순간 뒤집혀 바닥으로 떨어진다.

원인
    IsaacConveyor 노드는 매 물리 스텝마다 대상 프림의
    physxSurfaceVelocity:surfaceVelocity 를 덮어쓴다. 같은 프림을 노드 둘이
    가리키면 스텝마다 값이 엎치락뒤치락해서 박스를 튕겨낸다.

    GUI 의 Conveyor 확장으로 조각을 선택한 채 그래프를 만들면 /World 밑에
    그래프가 하나 더 생기는데, 여기엔 graph:variable:Velocity 도 없어서
    속도를 0 으로 읽는다. 그 상태로 Ctrl+S 하면 씬에 박제된다.

규칙
    (1) 조각(=/World 바로 밑 프림) 하나당 구동 노드는 하나여야 한다.
        타깃이 조각 자신이든 그 자식(Belt/Rollers)이든 전부 그 조각으로 묶어
        센다. 조각 밖에 사는 그래프가 그 조각을 미는 것이 중복이다.
        예외: 소터(A43)는 본선 롤러와 분기 롤러를 따로 미는 게 정상이라
        타깃이 서로 다르면 둘이어도 놔둔다.
    (2) 컨베이어 에셋(ConveyorBelt_*)이 아닌 프림을 미는 그래프는 전부
        지운다. GroundPlane 에 붙어 바닥을 컨베이어로 굴리고 있던 적이 있다.
    (3) 그래프를 지워도 끝이 아니다. IsaacConveyor 노드는 대상 프림에
        PhysicsRigidBodyAPI / PhysicsCollisionAPI / PhysxSurfaceVelocityAPI 를
        적용해 두는데, 이게 남으면 고정이어야 할 컨베이어 조각과 바닥이
        동적 강체가 되어 박스를 튕겨낸다. 아무 노드도 안 미는 프림에서
        이 스키마들을 떼어낸다.

    ./run_fix_dup_conveyor.sh          # 검사만
    PL_APPLY=1 ./run_fix_dup_conveyor.sh   # 실제로 지우고 저장
"""

import os
import shutil
import time

from pxr import Sdf

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
APPLY = bool(os.environ.get("PL_APPLY"))

layer = Sdf.Layer.FindOrOpen(USD_PATH)


def conveyor_nodes(prim, out):
    """(그래프 프림, 노드 프림, 대상 경로) 를 모은다."""
    for a in prim.properties:
        if a.name != "inputs:conveyorPrim":
            continue
        rel = layer.GetPropertyAtPath(a.path)
        if rel is None:
            continue
        for t in rel.targetPathList.GetAddedOrExplicitItems():
            out.append((prim.path, t))
    for c in prim.nameChildren:
        conveyor_nodes(c, out)


found = []
for root in layer.rootPrims:
    conveyor_nodes(root, found)

def is_conveyor(track_path):
    """그 조각이 실제 컨베이어 에셋을 참조하는가."""
    prim = layer.GetPrimAtPath(track_path)
    if prim is None or not prim.HasInfo("references"):
        return False
    refs = prim.GetInfo("references")
    for slot in ("prependedItems", "explicitItems", "appendedItems", "addedItems"):
        for r in getattr(refs, slot, []):
            if "ConveyorBelt_" in r.assetPath:
                return True
    return False


def owner_track(path):
    """경로를 /World 바로 밑 조각 경로로 접는다."""
    parts = str(path).split("/")
    if len(parts) >= 3 and parts[1] == "World":
        return Sdf.Path("/World/" + parts[2])
    return Sdf.Path("/" + parts[1]) if len(parts) >= 2 else path


by_track = {}
for node_path, target in found:
    by_track.setdefault(owner_track(target), []).append((node_path, target))

dupes = []
for track, entries in sorted(by_track.items(), key=lambda kv: str(kv[0])):
    mark = "" if len(entries) == 1 else "   ← 중복"
    print(f"{str(track):<32s} 구동 노드 {len(entries)}개{mark}")
    for node_path, target in entries:
        home = owner_track(node_path)
        where = "조각 안" if home == track else f"조각 밖 ({home})"
        print(f"      {where:<28s} {node_path}  ->  {target}")
    if not is_conveyor(track):
        print(f"      ↑ {track} 은 컨베이어 에셋이 아니다 — 전부 제거 대상")
        dupes.extend(n for n, _ in entries)
        continue
    if len(entries) < 2:
        continue
    if len({str(t) for _, t in entries}) == len(entries) and all(
            owner_track(n) == track for n, _ in entries):
        continue          # 소터: 본선 + 분기, 서로 다른 타깃이면 정상
    inside = [n for n, _ in entries if owner_track(n) == track]
    outside = [n for n, _ in entries if owner_track(n) != track]
    # 조각 안에 정상 그래프가 있을 때만 바깥 것을 지운다.
    # 안에 하나도 없으면 바깥 것이 유일한 구동원이므로 건드리지 않는다.
    if inside and outside:
        dupes.extend(outside)

if not dupes:
    print("\n중복 그래프 없음. 고아 스키마만 확인한다.")

# 지울 대상은 노드가 아니라 그 노드를 담은 그래프 프림 전체다.
graphs = set()
for node_path in dupes:
    p = node_path.GetParentPath()
    while p != Sdf.Path.absoluteRootPath:
        spec = layer.GetPrimAtPath(p)
        if spec is not None and spec.typeName == "OmniGraph":
            graphs.add(p)
            break
        p = p.GetParentPath()

if graphs:
    print(f"\n지울 그래프 {len(graphs)}개:")
    for g in sorted(graphs, key=str):
        print("   ", g)

if not APPLY:
    print("\n검사만 했다. 실제로 고치려면 PL_APPLY=1 로 다시 돌려라.")
    raise SystemExit(0)

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"\n백업: {backup}")

for g in sorted(graphs, key=str):
    parent = layer.GetPrimAtPath(g.GetParentPath())
    del parent.nameChildren[g.name]
    print(f"   제거 {g}")

# ── 고아가 된 물리 스키마 정리 ─────────────────────────────
# 남은 노드들이 미는 대상을 다시 모은다 (지운 그래프는 이제 안 잡힌다).
alive = []
for root in layer.rootPrims:
    conveyor_nodes(root, alive)
still_driven = {str(t) for _, t in alive}

CONVEYOR_APIS = ("PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
                 "PhysxSurfaceVelocityAPI")

orphans = []


def scan_orphans(prim):
    if prim.HasInfo("apiSchemas"):
        api = list(prim.GetInfo("apiSchemas").GetAddedOrExplicitItems())
        if "PhysicsRigidBodyAPI" in api and prim.path.pathString not in still_driven:
            orphans.append((prim, api))
    for c in prim.nameChildren:
        scan_orphans(c)


for root in layer.rootPrims:
    scan_orphans(root)

for prim, api in orphans:
    keep = [x for x in api if x not in CONVEYOR_APIS]
    op = Sdf.TokenListOp()
    op.prependedItems = keep
    prim.SetInfo("apiSchemas", op)
    for name in list(prim.properties.keys()):
        if name.startswith("physxSurfaceVelocity:"):
            del prim.properties[name]
    print(f"   스키마 정리 {prim.path}  {api} -> {keep or '없음'}")

layer.Save()
print(f"저장 완료: {USD_PATH}")
