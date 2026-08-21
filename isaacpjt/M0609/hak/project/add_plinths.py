#!/usr/bin/env python3
"""컨베이어 바닥과 지면 사이의 빈 틈에 받침대를 세운다.

A06/A03 (원래 벨트면 1780mm) 을 A05/A02 (770mm) 로 바꿔서 파묻힘은 없앴지만,
900mm 벨트면을 맞추려면 조각 원점이 지면 위 121~131mm 에 떠야 한다.
다리가 공중에 뜬 상태가 된다.

    ConveyorTrack 계열   바닥 z = 0.1307
    Sorter 계열          바닥 z = 0.1215

조각을 세로로 늘려서(scale z 약 1.17) 다리를 지면까지 내리는 방법도 있지만,
롤러 콜리전 실린더가 타원으로 변하고 A43 소터의 접촉면 정렬(0.9000)이
다시 깨진다. 받침대를 대는 쪽이 안전하다.

받침대는 각 조각의 자식으로 만든다. 그래야 조각의 회전을 그대로 따라가서
커브나 대각선 조각에서도 어긋나지 않는다.

    ./run_add_plinths.sh
    PL_INSET=0.75 ./run_add_plinths.sh    # 조각 폭의 75% 로 (기본 0.8)
"""

import os

STAGE = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
PLINTH = "Plinth"                 # 각 조각 아래에 만들 자식 이름
INSET = float(os.environ.get("PL_INSET", 0.80))   # 조각 footprint 대비 비율
GROUND_Z = 0.0

from isaacsim import SimulationApp
app = SimulationApp({"headless": True},
                    experience="/home/rokey/isaacsim/apps/isaacsim.exp.full.kit")

import shutil
import time

import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

backup = f"{STAGE}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(STAGE, backup)
print(f"백업: {backup}\n")

ctx = omni.usd.get_context()
ctx.open_stage(STAGE)
for _ in range(150):
    app.update()
stage = ctx.get_stage()

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"], useExtentsHint=False)

made, skipped = [], []
for prim in stage.GetPrimAtPath("/World").GetChildren():
    name = prim.GetName()
    if not (name.startswith("ConveyorTrack") or name.startswith("Sorter")):
        continue

    # 이미 있으면 지우고 다시 만든다
    old = stage.GetPrimAtPath(f"/World/{name}/{PLINTH}")
    if old:
        stage.RemovePrim(old.GetPath())

    # 조각의 로컬 bbox. 로컬로 재야 회전을 자식이 그대로 물려받는다.
    r = cache.ComputeLocalBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        skipped.append(f"{name} (bbox 없음)")
        continue
    mn, mx = r.GetMin(), r.GetMax()

    # 조각 원점의 월드 높이 = 지면까지의 거리
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    origin_z = xf.ExtractTranslation()[2]
    # 로컬 스케일 (조각마다 x 로 늘려놓은 것이 있다)
    sx = Gf.Vec3d(xf[0][0], xf[0][1], xf[0][2]).GetLength()
    sy = Gf.Vec3d(xf[1][0], xf[1][1], xf[1][2]).GetLength()
    sz = Gf.Vec3d(xf[2][0], xf[2][1], xf[2][2]).GetLength()

    gap = origin_z - GROUND_Z          # 메워야 할 높이 [m]
    if gap <= 0.001:
        skipped.append(f"{name} (틈 {gap*1000:.1f}mm)")
        continue

    # 로컬 좌표계에서 조각 바닥(mn[2]) 아래로 gap/sz 만큼 내려간다
    h_local = gap / sz
    cx = (mn[0] + mx[0]) / 2.0
    cy = (mn[1] + mx[1]) / 2.0
    w = (mx[0] - mn[0]) * INSET
    d = (mx[1] - mn[1]) * INSET

    path = f"/World/{name}/{PLINTH}"
    cube = UsdGeom.Cube.Define(stage, path)
    p = cube.GetPrim()
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.29, 0.31)])

    x = UsdGeom.Xformable(p)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(cx, cy, mn[2] - h_local / 2.0))
    x.AddScaleOp().Set(Gf.Vec3d(w, d, h_local))

    UsdPhysics.CollisionAPI.Apply(p)

    made.append((name, gap * 1000, w * sx, d * sy))

print(f"{'조각':<20} {'메운 높이':>10} {'받침 크기 (월드)':>22}")
for name, g, w, d in made:
    print(f"{name:<20} {g:>9.1f}mm   {w:.3f} x {d:.3f} m")
if skipped:
    print(f"\n건너뜀: {', '.join(skipped)}")

stage.GetRootLayer().Save()
print(f"\n저장 완료: {STAGE}")
app.close()
