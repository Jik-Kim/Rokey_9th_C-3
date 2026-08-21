#!/usr/bin/env python3
"""세 목적지 끝에 끝막이를 세운다.

분류는 되는데 박스가 라인 끝에서 그냥 떨어진다. 초록 스퍼는 _10 이 x=-2.263
에서 끝나고 그 뒤가 비어 있다 (롤러 _11 이 지워진 상태). 빨강 라인 _08_B 도
x=-2.352 에서 끝난다. 검증 때 Box_01_No4 가 (-0.687, 9.104, z=0.140) 즉
바닥으로 떨어졌다.

트럭·파렛트가 들어오기 전까지 박스를 잡아두는 임시 벽이다. 필요 없어지면
/World/EndStops 를 통째로 지우면 된다.

    ./run_add_endstops.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

ROOT = "/World/EndStops"
BELT_TOP = 0.900
WALL_T = 0.10        # 두께 (진행 방향)
WALL_H = 0.40        # 높이 — 가장 큰 5호(340mm) 보다 높게
MARGIN = 0.06        # 벨트 끝에서 살짝 안쪽

# (이름, 벨트 끝 x, 벨트 y 구간)
STOPS = [
    ("Blue",  -1.126, (4.972, 5.875)),    # _04 롤러 끝 — 파랑
    ("Green", -2.263, (8.758, 9.658)),    # _10 끝 — 초록
    ("Red",   -2.352, (12.319, 13.260)),  # _08_B 끝 — 빨강
]

import shutil
import time

from pxr import Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)


def define(path, type_name, api=None):
    spec = Sdf.CreatePrimInLayer(layer, Sdf.Path(path))
    spec.specifier = Sdf.SpecifierDef
    spec.typeName = type_name
    if api:
        spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(api))
    return spec


def put(spec, name, tn, value, uniform=False):
    var = Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying
    a = Sdf.AttributeSpec(spec, name, tn, var)
    a.default = value
    return a


root = layer.GetPrimAtPath(Sdf.Path(ROOT))
if root is not None:
    del layer.GetPrimAtPath(Sdf.Path("/World")).nameChildren["EndStops"]
    print(f"기존 {ROOT} 제거")
define(ROOT, "Xform")

for name, end_x, (y0, y1) in STOPS:
    path = f"{ROOT}/Stop_{name}"
    # 정적 콜라이더. 리지드바디를 안 붙이면 고정 물체가 된다.
    spec = define(path, "Cube", ["PhysicsCollisionAPI", "PhysxCollisionAPI"])
    put(spec, "size", Sdf.ValueTypeNames.Double, 1.0)
    put(spec, "extent", Sdf.ValueTypeNames.Float3Array,
        [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    put(spec, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
        [Gf.Vec3f(0.85, 0.85, 0.2)])
    put(spec, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, True)

    cx = end_x - MARGIN - WALL_T / 2.0
    cy = (y0 + y1) / 2.0
    cz = BELT_TOP + WALL_H / 2.0
    put(spec, "xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(cx, cy, cz))
    put(spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd, Gf.Quatd(1, 0, 0, 0))
    put(spec, "xformOp:scale", Sdf.ValueTypeNames.Double3,
        Gf.Vec3d(WALL_T, y1 - y0, WALL_H))
    put(spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray,
        ["xformOp:translate", "xformOp:orient", "xformOp:scale"], uniform=True)
    print(f"  Stop_{name:<6} x={cx:7.3f}  y {y0:.3f}..{y1:.3f}  "
          f"z {BELT_TOP:.3f}..{BELT_TOP+WALL_H:.3f}")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
