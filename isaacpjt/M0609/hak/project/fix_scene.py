#!/usr/bin/env python3
"""벨트 폭을 파렛트보다 넓히고, 스케일 때문에 생긴 어긋남을 잡는다.

폭
  EUR1 파렛트 실치수 1213 x 802. 장변을 폭 방향으로 실을 수 있어야 하므로
  벨트면을 1300mm 로 맞춘다 (여유 87mm).
  직선 조각의 로컬 Y 가 벨트 폭 방향이다. xformOpOrder 가
  [translate, orient, scale] 이라 스케일이 회전 전 로컬에서 먹는다.
  커브(A03)는 벨트면이 이미 1944~2034mm 라 건드리지 않는다. Y 로만 늘리면
  호가 타원이 되어 이웃 조각과 안 맞는다.

높이
  scale.z 가 1 이 아니면 벨트 상면이 900mm 에서 벗어난다. 1.0 으로 되돌린다.

음수 스케일
  scale.x 가 음수면 거울 반전이라 PhysX 콜리전 법선이 뒤집힐 수 있다.
  덮는 구간을 유지한 채 양수로 바꾼다. 조각이 반대 방향으로 뻗게 되므로
  원점을 그만큼 옮기고, 흐름이 바뀌지 않도록 inputs:direction 도 뒤집는다.

    ./run_fix_scene.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
CACHE = "/home/rokey/.local/share/Trash/files/H2017_test1/SubUSDs"

TARGET_BELT_W = 1.300
CURVE_ASSETS = {"ConveyorBelt_A03.usd"}

import shutil
import time

from pxr import Usd, UsdGeom, Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)

_cache = {}
def asset_dims(a):
    """(에셋 전체 길이, 벨트면 폭) — 스케일 1 기준."""
    if a not in _cache:
        st = Usd.Stage.Open(f"{CACHE}/{a}")
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        whole = bb.ComputeWorldBound(st.GetPrimAtPath("/World")).ComputeAlignedRange()
        bw = None
        for p in st.Traverse():
            if p.GetName() in ("Belt", "Rollers"):
                r = bb.ComputeWorldBound(p).ComputeAlignedRange()
                bw = r.GetMax()[1] - r.GetMin()[1]
                break
        _cache[a] = (whole.GetMax()[0] - whole.GetMin()[0], bw)
    return _cache[a]


tracks = []
def collect(p):
    s = layer.GetPrimAtPath(p)
    if (s and s.typeName == "Xform" and p.pathString.count("/") == 2
            and "ConveyorTrack" in p.name):
        tracks.append(p)
layer.Traverse(Sdf.Path("/"), collect)


def attr(path, prop):
    return layer.GetAttributeAtPath(Sdf.Path(path).AppendProperty(prop))


def asset_of(p):
    refs = layer.GetPrimAtPath(p).referenceList.prependedItems
    return refs[0].assetPath.rsplit("/", 1)[-1] if refs else None


print("[1] 음수 X 스케일 제거 (덮는 구간 · 흐름 유지)")
for p in tracks:
    sc = attr(p, "xformOp:scale")
    if sc is None or sc.default[0] >= 0:
        continue
    a = asset_of(p)
    length, _ = asset_dims(a)
    S = sc.default
    q = attr(p, "xformOp:orient").default
    rot = Gf.Rotation(Gf.Quatd(q.GetReal(), q.GetImaginary()))
    tr = attr(p, "xformOp:translate")
    o = tr.default
    shift = rot.TransformDir(Gf.Vec3d(S[0] * length, 0, 0))
    tr.default = type(o)(o[0] + shift[0], o[1] + shift[1], o[2])
    sc.default = type(S)(-S[0], S[1], S[2])
    for child in layer.GetPrimAtPath(p).nameChildren:
        if child.typeName != "OmniGraph":
            continue
        d = attr(p.AppendChild(child.name).AppendChild("ConveyorNode"), "inputs:direction")
        if d is not None and d.default is not None:
            v = d.default
            d.default = type(v)(-v[0], -v[1], -v[2])
    print(f"  {p.name:<20}scale.x {S[0]:+.3f} -> {-S[0]:+.3f}, 원점 {o[0]:.3f} -> "
          f"{tr.default[0]:.3f}, direction 반전")

print(f"\n[2] 벨트면 폭을 {TARGET_BELT_W*1000:.0f}mm 로 (커브 제외)")
for p in sorted(tracks, key=lambda x: x.name):
    a = asset_of(p)
    if a in CURVE_ASSETS:
        print(f"  {p.name:<20}커브 — 건너뜀")
        continue
    _, bw = asset_dims(a)
    sc = attr(p, "xformOp:scale")
    S = sc.default
    k = TARGET_BELT_W / bw
    sc.default = type(S)(S[0], k, S[2])
    print(f"  {p.name:<20}scale.y {S[1]:.3f} -> {k:.4f}   "
          f"폭 {bw*S[1]*1000:.0f} -> {bw*k*1000:.0f} mm")

print("\n[3] scale.z 를 1.0 으로 (벨트 상면 900mm 복귀)")
for p in sorted(tracks, key=lambda x: x.name):
    sc = attr(p, "xformOp:scale")
    S = sc.default
    if abs(S[2] - 1.0) < 1e-9:
        continue
    sc.default = type(S)(S[0], S[1], 1.0)
    print(f"  {p.name:<20}scale.z {S[2]:.3f} -> 1.0")

print("\n[4] 멈춰 있는 그래프에 Velocity 채우기")
for p in sorted(tracks, key=lambda x: x.name):
    for child in layer.GetPrimAtPath(p).nameChildren:
        if child.typeName != "OmniGraph":
            continue
        v = attr(p.AppendChild(child.name), "graph:variable:Velocity")
        if v is not None and not v.default:
            v.default = 0.5
            print(f"  {p.name}/{child.name}  Velocity {0.0} -> 0.5")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
