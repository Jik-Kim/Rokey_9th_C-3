#!/usr/bin/env python3
"""OmniGraph 의 교차 프림 참조를 자기 자신으로 되돌린다.

프림을 복사해서 컨베이어 조각을 늘리면 그래프 안의 경로가 원본을 그대로
가리킨다. 두 종류가 있는데 앞의 것만 고치면 조용히 죽는다.

    relationship  inputs:conveyorPrim  -> 어떤 롤러를 구동할지
    connection    inputs:velocity      -> read_speed.outputs:value

실제로 SorterRed 는 relationship 을 고친 뒤에도 velocity 연결이
/World/SorterGreen/ConveyorBeltGraph/read_speed 를 가리키고 있었다.
그래프 간 연결은 평가되지 않으므로 velocity 가 비고, 컨베이어 노드가
surfaceVelocity 를 아예 authoring 하지 않는다. Play 중 실측에서
SorterRed/Rollers 의 surfaceVelocity 가 없었고, 박스가 y=11.33
(SorterRed 롤러 시작점) 에서 멈췄다.

    ./run_fix_graph_refs.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
DEFAULT_VELOCITY = 0.5

import shutil
import time

from pxr import Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

tops = [c.name for c in layer.GetPrimAtPath("/World").nameChildren]


def retarget(path, owner):
    """/World/<다른조각>/... 를 /World/<owner>/... 로 바꾼다."""
    parts = path.pathString.split("/")
    if len(parts) > 3 and parts[1] == "World" and parts[2] in tops and parts[2] != owner:
        parts[2] = owner
        return Sdf.Path("/".join(parts))
    return path


def fix_list(lst, owner):
    """Sdf 리스트 편집의 세 슬롯을 모두 훑는다."""
    changed = []
    for slot in ("explicitItems", "prependedItems", "appendedItems", "addedItems"):
        try:
            items = list(getattr(lst, slot))
        except Exception:
            continue
        if not items:
            continue
        new = [retarget(p, owner) for p in items]
        if new != items:
            changed.append((slot, items, new))
            del getattr(lst, slot)[:]
            for p in new:
                getattr(lst, slot).append(p)
    return changed


total = 0
for owner in tops:
    root = Sdf.Path(f"/World/{owner}")
    hits = []

    def walk(path):
        global total
        spec = layer.GetPrimAtPath(path)
        if spec is None:
            return
        for prop in spec.properties:
            ppath = path.AppendProperty(prop.name)
            for kind, lst in (("connection", getattr(prop, "connectionPathList", None)),
                              ("target", getattr(prop, "targetPathList", None))):
                if lst is None:
                    continue
                for slot, old, new in fix_list(lst, owner):
                    hits.append(f"    {ppath.name:<24} {kind:<10} {old[0]} -> {new[0]}")
                    total += 1
        for child in spec.nameChildren:
            walk(path.AppendChild(child.name))

    walk(root)
    if hits:
        print(f"  {owner}")
        for h in hits:
            print(h)

print(f"\n교차 참조 {total}건 교정")

# ---- 컨베이어 속도 점검 ----
print("\n[그래프별 Velocity]")
for owner in tops:
    spec = layer.GetPrimAtPath(f"/World/{owner}")
    if spec is None:
        continue
    for child in spec.nameChildren:
        if child.typeName != "OmniGraph":
            continue
        a = layer.GetAttributeAtPath(
            Sdf.Path(f"/World/{owner}/{child.name}").AppendProperty("graph:variable:Velocity"))
        if a is None:
            continue
        if not a.default:
            # 조각을 새로 놓거나 복사하면 이 변수가 자주 빈 채로 남는다.
            # 비어 있으면 컨베이어 노드가 속도 0 을 써서 그 구간이 통째로 죽는다.
            a.default = DEFAULT_VELOCITY
            print(f"  {owner}/{child.name:<24} 비어 있음 -> {DEFAULT_VELOCITY}")
        else:
            print(f"  {owner}/{child.name:<24} {a.default}")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
