#!/usr/bin/env python3
"""분기점에 키네매틱 푸셔를 세운다.

왜 필요한가
  기존 sorter 는 박스의 physics:velocity 를 1.5 m/s 로 순간에 덮어썼다.
  바닥은 벨트 마찰로 붙잡혀 있는데 몸통만 튕겨나가니 토크가 생겨 박스가
  뒤집힌다. 실제로 파란색 분류에서 뒤집힘이 관측됐다.
  실물 밀대가 옆면을 밀면 접촉점에서 힘이 들어가 그 현상이 없다.

배치
  본선 레인  _05/Belt  x -9.946..-9.046  (분기 y 5.413)
             _06/Belt  x -9.946..-9.046  (분기 y 9.204)
  분기 레인  Belt_01   x -9.057..-7.046

  푸셔는 본선 서쪽(x -10.2)에 대기하다 +X 로 전진해 박스를 분기 레인에
  올린다. 박스 중심이 -9.05 를 넘으면 Belt_01 이 이어받는다.

높이
  밑면 905mm — 벨트면 900mm 에서 5mm 띄운다. 높이 200mm 라 접촉점이
  박스 밑면에서 100mm 다. 3~5호 무게중심(105~170mm)보다 낮아 덜 넘어간다.

    ./run_add_pushers.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

ROOT = "/World/Pushers"
BELT_TOP = 0.900
PLATE = (0.15, 0.60, 0.20)      # 두께(x) x 폭(y) x 높이(z)
GAP = 0.005                     # 벨트면과의 틈
HOME_X = -10.20
EXTEND_X = -9.30

# (이름, 분기 중심 y)
PUSHERS = [("Blue", 5.413), ("Green", 9.204)]

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


if layer.GetPrimAtPath(Sdf.Path(ROOT)) is not None:
    del layer.GetPrimAtPath(Sdf.Path("/World")).nameChildren["Pushers"]
    print(f"기존 {ROOT} 제거")
define(ROOT, "Xform")

for name, y in PUSHERS:
    path = f"{ROOT}/Pusher_{name}"
    spec = define(path, "Cube",
                  ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI",
                   "PhysicsCollisionAPI", "PhysxCollisionAPI"])
    put(spec, "size", Sdf.ValueTypeNames.Double, 1.0)
    put(spec, "extent", Sdf.ValueTypeNames.Float3Array,
        [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    put(spec, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
        [Gf.Vec3f(0.9, 0.5, 0.1)])
    # 키네매틱: 물리에 안 밀리고 스크립트가 위치를 준다.
    put(spec, "physics:rigidBodyEnabled", Sdf.ValueTypeNames.Bool, True)
    put(spec, "physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, True)
    put(spec, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, True)

    cz = BELT_TOP + GAP + PLATE[2] / 2.0
    put(spec, "xformOp:translate", Sdf.ValueTypeNames.Double3,
        Gf.Vec3d(HOME_X, y, cz))
    put(spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd, Gf.Quatd(1, 0, 0, 0))
    put(spec, "xformOp:scale", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*PLATE))
    put(spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray,
        ["xformOp:translate", "xformOp:orient", "xformOp:scale"], uniform=True)
    print(f"  Pusher_{name:<6} 대기 x={HOME_X}  전진 x={EXTEND_X}  "
          f"y={y}  z {BELT_TOP+GAP:.3f}..{BELT_TOP+GAP+PLATE[2]:.3f}")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
