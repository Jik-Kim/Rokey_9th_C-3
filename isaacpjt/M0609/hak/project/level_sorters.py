#!/usr/bin/env python3
"""A43 소터의 세 접촉면을 본선과 같은 0.9000 으로 맞춘다.

런타임 실측 (A43 참조는 오프라인에서 안 풀리므로 Isaac Sim 안에서 잰 값):
    이웃 벨트면                     0.9000
    Rollers            상면 0.8997   본선 롤러 46개
    Rollers_01         상면 0.8993   분기 롤러
    Sorter/Conveyor_physics 상면 0.9085   실린더 24개 (purpose=proxy)

데크가 8.5 mm 솟아 있어 박스가 소터마다 턱을 넘어야 한다.

한 번은 데크만 8.8 mm 내려봤는데 결과가 9/10 -> 1/10 으로 나빠졌다. 이유는
Rollers 의 y 배치를 재보고 알았다.

    롤러 46개, 평균 간격 87 mm, 그런데 최대 간격 892.9 mm
    데크 구간(y 4.65..5.69) 안의 롤러는 3개뿐
    데크 실린더는 y 4.933..5.629 를 채운다

즉 데크는 그 구간의 유일한 지지면이다. 내리면 턱이 없어지는 게 아니라
893 mm 짜리 구덩이가 생긴다.

그래서 데크는 원위치로 두고, 조각 전체를 내린 다음 롤러를 올려 세 면을
전부 0.9000 에 모은다.

    조각 원점  z 0.1300 -> 0.1215   (-8.5 mm)   => 데크 0.9085 -> 0.9000
    Rollers    로컬 z +0.0088                    => 0.8912 -> 0.9000
    Rollers_01 로컬 z +0.0092                    => 0.8908 -> 0.9000

    ./run_level_sorters.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

TARGET_TOP = 0.9000
ROOT_DROP = 0.0085          # 조각 원점을 내릴 양
ROLLER_LIFT = 0.0088        # 본선 롤러를 올릴 양 (로컬)
BRANCH_LIFT = 0.0092        # 분기 롤러를 올릴 양 (로컬)
DECK_SPEED = 0.5            # 턱이 없어지면 본선과 같은 속도로 충분하다

import shutil
import time

from pxr import Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)
sorters = [c.name for c in layer.GetPrimAtPath("/World").nameChildren
           if layer.GetPrimAtPath(Sdf.Path(f"/World/{c.name}/Sorter/ActionGraph"))]
print(f"소터: {sorters}\n")

# 에셋 원본 로컬 translate (런타임 실측). 이 레이어에 스펙이 없으면 이걸 쓴다.
ASSET_LOCAL = {
    "Rollers":                 Gf.Vec3d(2.0, 0.0, 0.0),
    "Rollers_01":              Gf.Vec3d(2.41936, -1.30575, 0.74036),
    "Sorter/Conveyor_physics": Gf.Vec3d(-0.41211867324828555, -0.000708905792234901, 0.0),
}
LIFT = {"Rollers": ROLLER_LIFT, "Rollers_01": BRANCH_LIFT,
        "Sorter/Conveyor_physics": 0.0}


def set_local_z(root, sub, z):
    path = Sdf.Path(f"/World/{root}/{sub}")
    prim = layer.GetPrimAtPath(path)
    if prim is None:
        prim = Sdf.CreatePrimInLayer(layer, path)
        prim.specifier = Sdf.SpecifierOver
    attr = layer.GetAttributeAtPath(path.AppendProperty("xformOp:translate"))
    if attr is None:
        attr = Sdf.AttributeSpec(prim, "xformOp:translate", Sdf.ValueTypeNames.Double3)
        attr.default = ASSET_LOCAL[sub]
    base = ASSET_LOCAL[sub]
    old = Gf.Vec3d(attr.default)
    attr.default = Gf.Vec3d(base[0], base[1], base[2] + z)
    order = layer.GetAttributeAtPath(path.AppendProperty("xformOpOrder"))
    if order is None:
        order = Sdf.AttributeSpec(prim, "xformOpOrder", Sdf.ValueTypeNames.TokenArray)
        order.default = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]
    return old[2], attr.default[2]


print(f"[1] 조각 원점 {ROOT_DROP*1000:.1f} mm 내림")
for name in sorters:
    a = layer.GetAttributeAtPath(Sdf.Path(f"/World/{name}").AppendProperty("xformOp:translate"))
    if a is None:
        print(f"  {name}: translate 없음 — 건너뜀")
        continue
    old = Gf.Vec3d(a.default)
    # 이미 내려간 상태에서 또 내리지 않도록 절대값으로 지정한다
    a.default = Gf.Vec3d(old[0], old[1], 0.1300 - ROOT_DROP)
    print(f"  {name}: z {old[2]:.4f} -> {a.default[2]:.4f}")

print(f"\n[2] 접촉면을 {TARGET_TOP:.4f} 로 정렬")
for name in sorters:
    for sub, lift in LIFT.items():
        o, n = set_local_z(name, sub, lift)
        print(f"  {name}/{sub:<24} 로컬 z {o:+.4f} -> {n:+.4f}")

print(f"\n[3] SorterSpeed -> {DECK_SPEED} (턱이 없으니 본선과 동일하게)")
for name in sorters:
    a = layer.GetAttributeAtPath(
        Sdf.Path(f"/World/{name}/Sorter/ActionGraph/SorterSpeed").AppendProperty("inputs:value"))
    if a is None:
        print(f"  {name}: 스펙 없음")
        continue
    old = a.default
    a.default = DECK_SPEED
    print(f"  {name}: {old} -> {DECK_SPEED}")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
