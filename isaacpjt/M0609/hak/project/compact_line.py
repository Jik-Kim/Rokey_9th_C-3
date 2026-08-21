#!/usr/bin/env python3
"""조각을 지워 생긴 빈 구간을 없애고 세로 본선을 아래로 당겨 붙인다.

벨트를 다시 채우는 게 아니라 남은 조각을 흐름 방향(+Y) 상류로 끌어와
맞물리게 한다. 그래야 라인이 실제로 짧아진다.

같이 따라가야 하는 것들
  · 스퍼는 자기가 분기하는 A24 조각과 같은 거리만큼 옮긴다.
      스퍼2 (_12_B/_12/_13/_04) <- _05
      스퍼1 (_10_B/_10/_11)     <- _06
  · 상단 라인 (_08_B/_08/_09) 은 커브 _07 과 같이 옮긴다.
  · 로봇과 받침대 Cube_01 은 스퍼2 와 같이 옮긴다 (픽 거리 유지).

조각이 덮는 구간은 회전을 반영해 계산한다. 세로 조각 중 _03 은 yaw +90
(원점에서 +Y 로 뻗음), 나머지는 yaw -90 (원점에서 -Y 로 뻗음) 이라
원점만 비교하면 안 된다.

    ./run_compact_line.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
CACHE = "/home/rokey/.local/share/Trash/files/H2017_test1/SubUSDs"

# 흐름 순서. _03 은 커브 _17 출구에 물려 있으므로 고정점으로 쓴다.
VERTICAL = ["ConveyorTrack_03", "ConveyorTrack_05",
            "ConveyorTrack_18", "ConveyorTrack_06"]
CURVE = "ConveyorTrack_07"
# 커브와 함께 움직이는 상단 라인
TOP = ["ConveyorTrack_08_B", "ConveyorTrack_08", "ConveyorTrack_09"]
# A24 -> 딸린 스퍼
SPUR_OF = {
    "ConveyorTrack_05": ["ConveyorTrack_12_B", "ConveyorTrack_12",
                         "ConveyorTrack_13", "ConveyorTrack_04"],
    "ConveyorTrack_06": ["ConveyorTrack_10_B", "ConveyorTrack_10",
                         "ConveyorTrack_11"],
}
# 스퍼2 를 따라가는 것들
FOLLOW_SPUR2 = ["robot", "Cube_01"]

import shutil
import time

from pxr import Usd, UsdGeom, Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

_len = {}
def asset_len(asset):
    if asset not in _len:
        st = Usd.Stage.Open(f"{CACHE}/{asset}")
        c = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        r = c.ComputeWorldBound(st.GetPrimAtPath("/World")).ComputeAlignedRange()
        _len[asset] = r.GetMax()[0] - r.GetMin()[0]
    return _len[asset]


def track(name):
    return Sdf.Path(f"/World/{name}")


def xf(name):
    p = track(name)
    t = layer.GetAttributeAtPath(p.AppendProperty("xformOp:translate"))
    q = layer.GetAttributeAtPath(p.AppendProperty("xformOp:orient"))
    return t, (q.default if q else None)


def span_y(name):
    """조각이 덮는 y 구간 (시작, 끝)."""
    t, q = xf(name)
    spec = layer.GetPrimAtPath(track(name))
    asset = spec.referenceList.prependedItems[0].assetPath.rsplit("/", 1)[-1]
    rot = Gf.Rotation(Gf.Quatd(q.GetReal(), q.GetImaginary()))
    e = rot.TransformDir(Gf.Vec3d(asset_len(asset), 0, 0))
    y0, y1 = sorted([t.default[1], t.default[1] + e[1]])
    return y0, y1


def shift_y(name, dy):
    p = track(name)
    a = layer.GetAttributeAtPath(p.AppendProperty("xformOp:translate"))
    if a is None:
        print(f"    [translate 없음] {name}")
        return
    o = a.default
    a.default = type(o)(o[0], o[1] + dy, o[2])


print("[1] 세로 본선 압축")
cursor = None
deltas = {}
for name in VERTICAL:
    y0, y1 = span_y(name)
    if cursor is None:              # _03 은 고정
        cursor = y1
        print(f"  {name:<20}{y0:>8.3f} .. {y1:>8.3f}   고정 (커브 _17 출구)")
        continue
    dy = cursor - y0
    shift_y(name, dy)
    deltas[name] = dy
    ny0, ny1 = span_y(name)
    print(f"  {name:<20}{y0:>8.3f} .. {y1:>8.3f}  ->{ny0:>8.3f} .. {ny1:>8.3f}   ({dy:+.3f})")
    cursor = ny1

print("\n[2] 스퍼를 분기 조각과 같이 이동")
spur2_dy = 0.0
for a24, spur in SPUR_OF.items():
    dy = deltas.get(a24, 0.0)
    if a24 == "ConveyorTrack_05":
        spur2_dy = dy
    if abs(dy) < 1e-9:
        continue
    for s in spur:
        if layer.GetPrimAtPath(track(s)) is None:
            print(f"    [없음] {s}")
            continue
        shift_y(s, dy)
    print(f"  {a24} 기준 {dy:+.3f}  ->  {', '.join(spur)}")

print("\n[3] 커브 _07 + 상단 라인 이동")
cy0, cy1 = span_y(CURVE)
t_curve, _ = xf(CURVE)
old_entry = t_curve.default[1]
dy_curve = cursor - old_entry
shift_y(CURVE, dy_curve)
print(f"  {CURVE:<20}진입 {old_entry:>8.3f} -> {old_entry+dy_curve:>8.3f}   ({dy_curve:+.3f})")
for name in TOP:
    t, _ = xf(name)
    before = t.default[1]
    shift_y(name, dy_curve)
    print(f"  {name:<20}y {before:>8.3f} -> {before+dy_curve:>8.3f}")

print("\n[4] 로봇 / 받침대를 스퍼2 와 같이 이동")
for name in FOLLOW_SPUR2:
    if layer.GetPrimAtPath(track(name)) is None:
        print(f"  [없음] {name}")
        continue
    t, _ = xf(name)
    before = t.default[1]
    shift_y(name, spur2_dy)
    print(f"  {name:<20}y {before:>8.3f} -> {before+spur2_dy:>8.3f}   ({spur2_dy:+.3f})")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
