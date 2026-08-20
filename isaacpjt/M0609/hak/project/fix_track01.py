#!/usr/bin/env python3
"""ConveyorTrack / ConveyorTrack_01 구간이 큐브를 못 넘기는 문제를 고친다.

두 가지를 잡는다.
  1. ConveyorTrack_01 의 Velocity 가 0 으로 남아 있다 -> 0.5
  2. ConveyorTrack / ConveyorTrack_01 (둘 다 ConveyorBelt_A30) 이 라인 하류인
     -X 가 아니라 +X 로 민다 -> inputs:direction 을 (-1,0,0) 으로.

방향 계산 주의: 월드 진행 방향은 Track 의 회전만으로 안 나온다.
ConveyorBelt_A30.usd 안의 /World/Belt 프림 자체가 180 deg/Z 회전을 갖고 있어서

    world = R_track * R_belt * inputs:direction

이고, A30 은 R_track(180) * R_belt(180) = identity 라 direction 이 그대로
월드 방향이 된다. A26/A06/A37 은 R_belt 가 identity 라 헷갈리기 쉽다.

라인 하류가 -X 인 근거는 커브 두 개다. _02(x=-8) 와 _07 모두 wz=-18.5, 즉
위에서 봤을 때 시계방향이다. y=0 라인을 서쪽으로 -> 북쪽(+Y) 은 우회전,
+Y -> 동쪽(+X) 도 우회전이라 둘 다 시계방향과 맞는다. 반대로 잡으면
좌회전이 되어 wz 부호와 어긋난다.

surfaceVelocity 는 그래프가 안 돌 때의 정적 폴백이라 같이 맞춰준다
(이쪽은 Belt 로컬 프레임이므로 direction * velocity 를 그대로 넣는다).

    ./run_fix_track01.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
VELOCITY = 0.5
DIRECTION = (-1.0, 0.0, 0.0)

import shutil
import time

from pxr import Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)
if layer is None:
    raise SystemExit(f"레이어를 못 열었다: {USD_PATH}")


def put(path, value):
    attr = layer.GetAttributeAtPath(Sdf.Path(path))
    if attr is None:
        print(f"  [없음] {path}")
        return
    old = attr.default
    mark = "" if old == value else "  *"
    print(f"  {old} -> {value}   {path}{mark}")
    attr.default = value


surf = Gf.Vec3f(*(c * VELOCITY for c in DIRECTION))

for track in ("ConveyorTrack", "ConveyorTrack_01"):
    print(f"[{track}]")
    put(f"/World/{track}/ConveyorBeltGraph.graph:variable:Velocity", VELOCITY)
    put(f"/World/{track}/ConveyorBeltGraph/ConveyorNode.inputs:direction", Gf.Vec3f(*DIRECTION))
    put(f"/World/{track}/Belt.physxSurfaceVelocity:surfaceVelocity", surf)
    print()

layer.Save()
print(f"저장 완료: {USD_PATH}")
