#!/usr/bin/env python3
"""본선 양쪽에 가이드 레일을 세운다.

단독 박스는 소터 세 개를 x 편차 1 mm 로 직진 통과한다. 그런데 10개를 3초
간격으로 흘리면 뒤차가 앞차를 밀면서 옆으로 튕겨나가고, 그렇게 동쪽으로
밀린 박스가 분기 롤러에 올라타 라인을 이탈한다. 실제 컨베이어 라인이
사이드 가드를 다는 이유가 이것이다.

배치 (런타임 실측 기준)
    벨트 폭        x -9.95 .. -9.05
    분기 롤러 시작 x -8.988
    벨트면         z 0.900

    서쪽 레일  x -9.99  전 구간 연속
    동쪽 레일  x -9.015 분기 창 4곳을 비우고 토막으로

분기 창(각 소터의 Rollers_01 이 본선에 붙는 y 구간)은 열어둬야 나중에
색상 분류를 켰을 때 박스가 빠져나갈 수 있다. 다만 분류를 아직 안 켠
상태에서는 그 창으로 박스가 새므로, 통과 검증 때는 전부 막는다.

    ./run_add_guiderails.sh            # 분기 창 개방 (분류용)
    PL_RAIL_FULL=1 ./run_add_guiderails.sh   # 전부 막음 (통과 검증용)
"""

import os

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
ROOT = "/World/GuideRails"

BELT_TOP = 0.900
RAIL_H = 0.200          # 벨트면 위로 올라오는 높이
RAIL_T = 0.030          # 두께

WEST_X = -9.990
EAST_X = -9.015

Y0, Y1 = 3.490, 15.230  # 소터 구간 전체 (SorterBlue 시작 ~ ConveyorTrack_01 시작)

# 각 소터의 분기 롤러가 본선에 붙는 y 구간 — 동쪽 레일을 비운다
BRANCH_WINDOWS = [
    (4.848, 7.084),     # SorterBlue
    (8.830, 11.066),    # SorterGreen
    (12.590, 14.827),   # SorterRed
]
if os.environ.get("PL_RAIL_FULL"):
    BRANCH_WINDOWS = []

import shutil
import time

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

stage = Usd.Stage.Open(USD_PATH)

# 다시 돌려도 되도록 기존 것을 지운다
if stage.GetPrimAtPath(ROOT):
    stage.RemovePrim(ROOT)
    print(f"기존 {ROOT} 제거")
UsdGeom.Xform.Define(stage, ROOT)


def rail(name, x, y0, y1):
    path = f"{ROOT}/{name}"
    cube = UsdGeom.Cube.Define(stage, path)
    prim = cube.GetPrim()
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.85, 0.15)])

    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, (y0 + y1) / 2.0, BELT_TOP + RAIL_H / 2.0))
    xf.AddScaleOp().Set(Gf.Vec3d(RAIL_T, y1 - y0, RAIL_H))

    # 정적 콜라이더 — RigidBodyAPI 를 붙이지 않는다
    UsdPhysics.CollisionAPI.Apply(prim)
    print(f"  {name:<16} x={x:+.3f}  y {y0:+7.3f}..{y1:+7.3f}  길이 {y1-y0:5.3f} m")


print(f"서쪽 레일 (연속)")
rail("Rail_West", WEST_X, Y0, Y1)

print(f"\n동쪽 레일 (분기 창 {len(BRANCH_WINDOWS)}곳 개방)")
segments = []
cursor = Y0
for w0, w1 in sorted(BRANCH_WINDOWS):
    if w0 > cursor:
        segments.append((cursor, w0))
    cursor = max(cursor, w1)
if cursor < Y1:
    segments.append((cursor, Y1))

for i, (a, b) in enumerate(segments):
    rail(f"Rail_East_{i:02d}", EAST_X, a, b)

print(f"\n개방 구간 (분기 출구)")
for w0, w1 in BRANCH_WINDOWS:
    print(f"  y {w0:+7.3f}..{w1:+7.3f}")

stage.GetRootLayer().Save()
print(f"\n저장 완료: {USD_PATH}")
