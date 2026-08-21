#!/usr/bin/env python3
"""분기점의 A24 T_MERGE 를 A43 Y_MERGE 소터로 교체한다.

A43 을 고른 이유
  본선이 +Y 로 흐르고 스퍼가 +X 쪽인데, 회전만으로 이 조합을 만들려면
  분기가 로컬 -Y 여야 한다.
      +90 deg 회전:  로컬 +X -> 월드 +Y (본선),  로컬 -Y -> 월드 +X (분기)
  A47(T_MERGE)은 분기가 로컬 +Y 라 회전시키면 스퍼가 없는 -X 쪽으로 나간다.

  A43 은 단일 롤러 데크(상면 770mm)라 z=+0.130 이면 롤러면이 정확히 900mm 다.
  A46 은 롤러/벨트 2단이라 한쪽 데크가 지하로 내려간다.

  Sorter 기구가 z 597..780 -> 월드 727..910 이라 휠이 롤러면 위로 10mm 솟는다.

배치 (에셋 로컬 실측)
  Rollers     x 0.009..3.991  y -0.450..0.453   상면 z 0.770
  Rollers_01  x 1.360..3.596  y -2.164..-0.508
  -> +90 deg 회전 후 월드
     본선  x X0-0.453..X0+0.450   y Y0+0.009..Y0+3.991
     분기  x X0+0.508..X0+2.164   y Y0+1.360..Y0+3.596  (중심 Y0+2.478)

    ./run_install_sorters.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
ASSET = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
         "/Assets/Isaac/5.1/Isaac/Props/Conveyors/ConveyorBelt_A43.usd")

LINE_X = -9.496
Z = 0.130                 # 0.900 - 0.770
CHAIN_START = 3.497       # _03 끝
MAIN_LEN = 3.982          # 0.009..3.991
BRANCH_MID = 2.478        # 원점에서 분기 중심까지 (로컬 x)
VELOCITY = 0.5

REMOVE = ["ConveyorTrack_05", "ConveyorTrack_06", "ConveyorTrack_18"]

# 소터 (이름, 기존 스퍼 중심 y, 스퍼에 딸린 프림들)
SORTERS = [
    ("SorterBlue", 5.416,
     ["ConveyorTrack_12", "ConveyorTrack_04", "EndStops/Stop_Blue",
      "robot", "Cube_01"]),
    ("SorterGreen", 9.208,
     ["ConveyorTrack_10", "EndStops/Stop_Green"]),
]
# 커브 이후 (A43 #2 끝에 맞춰 같이 이동)
DOWNSTREAM = ["ConveyorTrack_07", "ConveyorTrack_08_B", "EndStops/Stop_Red"]

import math
import shutil
import time

from pxr import Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)
world = layer.GetPrimAtPath(Sdf.Path("/World"))


def attr(path, prop):
    return layer.GetAttributeAtPath(Sdf.Path(path).AppendProperty(prop))


def shift(path, dy, dx=0.0):
    a = attr(f"/World/{path}", "xformOp:translate")
    if a is None or a.default is None:
        print(f"    [translate 없음] {path}")
        return
    o = a.default
    a.default = type(o)(o[0] + dx, o[1] + dy, o[2])


print("[1] 기존 분기 조각 제거")
for name in REMOVE:
    if layer.GetPrimAtPath(Sdf.Path(f"/World/{name}")) is not None:
        del world.nameChildren[name]
        print(f"  {name} 제거")

print("\n[2] A43 소터 배치 + 스퍼/로봇 이동")
y0 = CHAIN_START - 0.009
placed = []
for (name, old_mid, followers), _ in zip(SORTERS, range(len(SORTERS))):
    new_mid = y0 + BRANCH_MID
    dy = new_mid - old_mid

    spec = Sdf.CreatePrimInLayer(layer, Sdf.Path(f"/World/{name}"))
    spec.specifier = Sdf.SpecifierDef
    spec.typeName = "Xform"
    spec.referenceList.prependedItems = [Sdf.Reference(ASSET)]

    half = math.radians(90.0) / 2.0
    for pname, tn, val in (
        ("xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(LINE_X, y0, Z)),
        ("xformOp:orient", Sdf.ValueTypeNames.Quatd,
         Gf.Quatd(math.cos(half), Gf.Vec3d(0, 0, math.sin(half)))),
        ("xformOp:scale", Sdf.ValueTypeNames.Double3, Gf.Vec3d(1, 1, 1)),
    ):
        a = Sdf.AttributeSpec(spec, pname, tn)
        a.default = val
    a = Sdf.AttributeSpec(spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray,
                          Sdf.VariabilityUniform)
    a.default = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]

    print(f"  {name}  원점 ({LINE_X}, {y0:.3f}, {Z})  "
          f"본선 y {y0+0.009:.3f}..{y0+3.991:.3f}  분기 중심 y {new_mid:.3f}")
    print(f"    스퍼 이동 {dy:+.3f}: {', '.join(followers)}")
    for f in followers:
        shift(f, dy)
    placed.append((name, y0, new_mid, dy))
    y0 += MAIN_LEN

main_end = y0 + 0.009
print(f"\n[3] 커브 이후를 A43 #2 끝(y {main_end:.3f})에 맞춰 이동")
curve = attr("/World/ConveyorTrack_07", "xformOp:translate")
d_down = main_end - curve.default[1]
for name in DOWNSTREAM:
    shift(name, d_down)
print(f"  {d_down:+.3f}: {', '.join(DOWNSTREAM)}")

# 초록 스퍼는 분기 끝(x -7.332)에 맞춰 x 도 당긴다
GREEN_DX = -0.115
shift("ConveyorTrack_10", 0.0, GREEN_DX)
shift("EndStops/Stop_Green", 0.0, GREEN_DX)
print(f"  초록 스퍼 x {GREEN_DX:+.3f} (분기 끝 -7.332 에 맞춤)")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
print("\n다음: run_wire_sorters.sh 로 A43 롤러 구동 그래프를 붙여야 한다.")
