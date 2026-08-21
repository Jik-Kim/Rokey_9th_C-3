#!/usr/bin/env python3
"""경사진 벨트를 전부 없애고 모든 컨베이어 상면을 트럭 적재 높이에 맞춘다.

경사의 정체
  회전으로 기울어진 트랙은 없다. 경사는 ConveyorBelt_A37 조각(_08/_10/_12)이다.
  translate z 만 보면 Belt=756 / BeltRamp=1273 이라 평면 두 장처럼 보이지만,
  실제 지오메트리 bbox 를 재면 두 프림 다 경사면이다.

      A37  Belt      z  736.8 .. 1780.5 mm   <- 1043mm 를 타고 오르는 램프
      A37  BeltRamp  z  737.6 .. 1761.3 mm
      A06  Belt      z 1741.5 .. 1780.5 mm   <- 두께 19.5, 평면
      A05  Rollers   z  711.4 ..  769.3 mm   <- 두께 29.3, 평면

  즉 BeltRamp 만 꺼서는 경사가 안 없어진다. A37 조각 자체를 치워야 한다.

교체 방식
  A37 길이 3907mm, A06 길이 2001mm, 벨트 폭은 둘 다 900mm (y ±450) 로 같다.
  그래서 A37 한 장을 A06 두 장(합 4002mm)으로 바꾼다. 95mm 길어지지만
  조각이 서로 맞물리는 방향이라 문제되지 않는다.
  두 번째 장은 원본 트랙 스펙을 통째로 복사해 만든다. 복사본 안의 절대경로
  (그래프 연결·릴레이션십)를 새 이름으로 다시 써 줘야 한다.

높이
  FLAT_TOP_H = 0.900 = 트럭 적재함 바닥 지상고. 측면 윙바디에 직접 싣는
  방식이라 컨베이어 토출 높이 = 트럭 바닥 높이여야 한다.
  상면값은 추정하지 않고 에셋 bbox 로 실측한 값을 쓴다.

    ./run_flatten_line.sh
"""

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"
ASSET_CACHE = "/home/rokey/.local/share/Trash/files/H2017_test1/SubUSDs"

FLAT_TOP_H = 0.900

# 에셋 bbox 실측 벨트 상면 [m] (에셋 로컬 z)
MEASURED_TOP = {
    "ConveyorBelt_A03.usd": 1.7814,
    "ConveyorBelt_A06.usd": 1.7805,
    "ConveyorBelt_A24.usd": 1.7805,
    "ConveyorBelt_A05.usd": 0.7693,
}
# 경사 조각 -> 대체 조각, 대체 조각 길이 [m]
SLOPED = {"ConveyorBelt_A37.usd": ("ConveyorBelt_A06.usd", 2.001)}

ASSET_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
             "/Assets/Isaac/5.1/Isaac/Props/Conveyors/")

import shutil
import time

from pxr import Gf, Sdf

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

layer = Sdf.Layer.FindOrOpen(USD_PATH)
if layer is None:
    raise SystemExit(f"레이어를 못 열었다: {USD_PATH}")


def asset_of(prim_path):
    spec = layer.GetPrimAtPath(prim_path)
    refs = spec.referenceList.prependedItems if spec else []
    return refs[0].assetPath.rsplit("/", 1)[-1] if refs else None


def set_reference(prim_path, asset):
    spec = layer.GetPrimAtPath(prim_path)
    spec.referenceList.prependedItems = [Sdf.Reference(ASSET_URL + asset)]


def rewrite_paths(prim_path, old, new):
    """복사한 서브트리 안의 절대경로를 새 트랙 이름으로 고친다."""
    prim = layer.GetPrimAtPath(prim_path)
    if prim is None:
        return
    for prop in prim.properties:
        for listop in (getattr(prop, "connectionPathList", None),
                       getattr(prop, "targetPathList", None)):
            if listop is None:
                continue
            for field in ("explicitItems", "prependedItems", "appendedItems"):
                items = list(getattr(listop, field))
                if not items:
                    continue
                fixed = [Sdf.Path(str(p).replace(old, new, 1)) for p in items]
                if fixed != items:
                    setattr(listop, field, fixed)
    for child in prim.nameChildren:
        rewrite_paths(prim_path.AppendChild(child.name), old, new)


def drop_ramp_specs(prim_path):
    """A37 전용 스펙을 지운다.

    active=False 로 꺼두면 안 된다. 대체 에셋 A06 에는 BeltRamp 가 아예
    없어서, 꺼진 스펙을 되살리면 없는 프림을 가리키는 그래프가 남는다.
    스펙 자체를 제거한다."""
    parent = layer.GetPrimAtPath(prim_path)
    for name in ("ConveyorBeltGraph_01", "BeltRamp"):
        if layer.GetPrimAtPath(prim_path.AppendChild(name)) is None:
            continue
        del parent.nameChildren[name]
        print(f"    A37 전용 스펙 제거  {prim_path.name}/{name}")


tracks = []


def collect(p):
    spec = layer.GetPrimAtPath(p)
    if (spec and spec.typeName == "Xform"
            and p.pathString.count("/") == 2 and "ConveyorTrack" in p.name):
        tracks.append(p)


layer.Traverse(Sdf.Path("/"), collect)

print("[1] 경사 조각(A37) -> 평벨트(A06) 2장으로 교체")
made = []
for t in list(tracks):
    asset = asset_of(t)
    if asset not in SLOPED:
        continue
    repl, length = SLOPED[asset]
    print(f"  {t.name}  {asset} -> {repl} x2")
    drop_ramp_specs(t)

    # 두 번째 장을 만든다. 원본을 복사한 뒤 내부 절대경로를 고친다.
    dst = Sdf.Path(f"/World/{t.name}_B")
    if layer.GetPrimAtPath(dst) is not None:
        layer.RemovePrimIfInert(layer.GetPrimAtPath(dst))
    Sdf.CopySpec(layer, t, layer, dst)
    rewrite_paths(dst, f"/World/{t.name}/", f"/World/{t.name}_B/")
    drop_ramp_specs(dst)

    for path in (t, dst):
        set_reference(path, repl)

    # 두 번째 장을 조각 길이만큼 진행 방향으로 민다.
    q = layer.GetAttributeAtPath(t.AppendProperty("xformOp:orient")).default
    rot = Gf.Rotation(Gf.Quatd(q.GetReal(), q.GetImaginary()))
    step = rot.TransformDir(Gf.Vec3d(length, 0, 0))
    tr = layer.GetAttributeAtPath(t.AppendProperty("xformOp:translate"))
    base = tr.default
    dst_tr = layer.GetAttributeAtPath(dst.AppendProperty("xformOp:translate"))
    dst_tr.default = type(base)(base[0] + step[0], base[1] + step[1], base[2])
    print(f"    {dst.name} 배치  ({dst_tr.default[0]:.3f}, {dst_tr.default[1]:.3f})")
    made.append(dst)

tracks.extend(made)

print(f"\n[2] 모든 벨트 상면을 {FLAT_TOP_H*1000:.0f}mm 로 정렬")
skipped = []
for t in sorted(tracks, key=lambda p: (len(p.name), p.name)):
    asset = asset_of(t)
    top = MEASURED_TOP.get(asset)
    if top is None:
        skipped.append((t.name, f"실측 상면 없음 ({asset})"))
        continue
    new_z = FLAT_TOP_H - top
    attr = layer.GetAttributeAtPath(t.AppendProperty("xformOp:translate"))
    old = attr.default
    attr.default = type(old)(old[0], old[1], new_z)
    print(f"  {t.name:<20}{asset.replace('ConveyorBelt_','').replace('.usd',''):<5}"
          f"상면 {top*1000:>7.1f} -> z {new_z:>7.4f}  (월드 상면 {(new_z+top)*1000:.1f})")

layer.Save()
print(f"\nA37 {len(made)}장 교체, 트랙 {len(tracks)}개 정렬")
if skipped:
    print("\n건너뜀:")
    for n, why in skipped:
        print(f"  {n:<20}{why}")
print(f"\n저장 완료: {USD_PATH}")
