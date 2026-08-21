#!/usr/bin/env python3
"""컬럼 중간에서 박스가 멈추는 원인 — 죽은 컨베이어 트랙을 살린다.

    ./run_fix_dead_tracks.sh

증상
    투입한 박스가 본선 컬럼(x=-9.496)을 북상하다 y=9~11 (ConveyorTrack_18) 과
    y=17~21 (ConveyorTrack_22/_21) 에서 서거나 뒤로 밀리며 걸린다.

원인
    ConveyorTrack_18 / _20 의 그래프 변수 Velocity 가 "선언만 되고 값이 없다".
        custom float graph:variable:Velocity (          <- 기본값 없음
    read_speed(ReadVariable) 가 0 을 읽어 ConveyorNode 가 표면속도 0 을 준다.
    같은 이유로 set_conveyor_offline.py 도 이 둘을 건너뛴다 (attr.default 가 None).
    게다가 정상 트랙에 다 있는 over "Belt" (PhysxSurfaceVelocityAPI) 정적 폴백도 없다.

    _21/_22 는 Velocity 는 0.5 로 멀쩡한데 방향 부호만 반대다. 북상하는 박스를
    남쪽으로 되밀기 때문에 "걸리는" 느낌이 난다 (멈추는 게 아니라 밀고 당긴다).

    방향 부호가 틀린 근거. 이 컬럼은 -90도 회전(quat w=0.707, z=-0.707)이라
    로컬 -X 가 월드 +Y 다. 정상인 _15/_19 는 direction=(-1,0,0) 인데
    _18/_20 만 (1,0,0) 이라, 값을 넣어도 남쪽으로 밀게 된다.

고치는 것 (정상 이웃 _15/_19 와 동일하게 맞춘다)
    1. graph:variable:Velocity = 0.5
    2. ConveyorNode.inputs:direction = (-1, 0, 0)
    3. over "Belt" 에 PhysxSurfaceVelocityAPI + surfaceVelocity=(-0.5,0,0) 추가
"""

import shutil
import time

from pxr import Gf, Sdf

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

# 컬럼(x=-9.496, -90도 회전) 구간에서 고장난 트랙.
#   _18/_20 : Velocity 값 없음 + 방향 부호 반대
#   _21/_22 : Velocity 는 0.5 로 정상인데 방향 부호만 반대 (역방향으로 민다)
DEAD_TRACKS = ("ConveyorTrack_18", "ConveyorTrack_20",
               "ConveyorTrack_21", "ConveyorTrack_22")
VELOCITY = 0.5
DIRECTION = Gf.Vec3f(-1.0, 0.0, 0.0)          # -90도 회전 컬럼 -> 월드 +Y
SURFACE_VEL = Gf.Vec3f(-0.5, 0.0, 0.0)


def main():
    backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
    shutil.copy2(USD_PATH, backup)
    print(f"백업: {backup}\n")

    layer = Sdf.Layer.FindOrOpen(USD_PATH)
    if layer is None:
        raise SystemExit(f"레이어를 못 열었다: {USD_PATH}")

    for track in DEAD_TRACKS:
        print(f"[{track}]")
        root = Sdf.Path(f"/World/{track}")
        if layer.GetPrimAtPath(root) is None:
            print("  프림이 없다. 건너뛴다.\n")
            continue

        put(layer, root.AppendPath("ConveyorBeltGraph.graph:variable:Velocity"), VELOCITY)
        put(layer, root.AppendPath("ConveyorBeltGraph/ConveyorNode.inputs:direction"), DIRECTION)
        add_surface_velocity(layer, root.AppendChild("Belt"))
        print()

    layer.Save()
    print(f"저장 완료: {USD_PATH}")


def put(layer, path, value):
    attr = layer.GetAttributeAtPath(path)
    if attr is None:
        print(f"  [속성 없음] {path}")
        return
    old = attr.default
    attr.default = value
    print(f"  {old} -> {value}   {path.name}")


def add_surface_velocity(layer, belt_path):
    """참조된 Belt 프림 위에 over 를 만들어 정적 표면속도 폴백을 준다.

    그래프가 안 도는 환경(헤드리스, 확장 미로딩)에서도 벨트가 돈다.
    """
    spec = layer.GetPrimAtPath(belt_path)
    if spec is None:
        spec = Sdf.CreatePrimInLayer(layer, belt_path)
        spec.specifier = Sdf.SpecifierOver

    schemas = list(spec.GetInfo("apiSchemas").prependedItems) if spec.HasInfo("apiSchemas") else []
    if "PhysxSurfaceVelocityAPI" not in schemas:
        schemas.append("PhysxSurfaceVelocityAPI")
        spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=schemas))
        print(f"  apiSchemas += PhysxSurfaceVelocityAPI   {belt_path.name}")

    for name, type_name, value in (
        ("physxSurfaceVelocity:surfaceVelocity", Sdf.ValueTypeNames.Vector3f, SURFACE_VEL),
        ("physxSurfaceVelocity:surfaceVelocityEnabled", Sdf.ValueTypeNames.Bool, True),
    ):
        attr = spec.properties.get(name)
        if attr is None:
            attr = Sdf.AttributeSpec(spec, name, type_name)
        old = attr.default
        attr.default = value
        print(f"  {old} -> {value}   {belt_path.name}.{name.split(':')[-1]}")


if __name__ == "__main__":
    main()
