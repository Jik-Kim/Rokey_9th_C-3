"""
최소 확인용 : 상자 스폰 -> 3D 인식(치수) -> 팔레트 적재
Isaac Sim Script Editor:
    exec(open("/home/rokey/min_pack.py").read())
"""

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom

try:
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, GroundPlane
except ImportError:
    from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane

# 팔레트 T11
PW, PD, PH = 1.10, 1.10, 0.15

# 상자 5개 (L, W, H)
SPEC = [
    ("box_0", 0.34, 0.25, 0.21),
    ("box_1", 0.34, 0.25, 0.21),
    ("box_2", 0.27, 0.18, 0.15),
    ("box_3", 0.41, 0.31, 0.28),
    ("box_4", 0.22, 0.19, 0.09),
]

stage = omni.usd.get_context().get_stage()

# --- 1. 씬 ---
GroundPlane(prim_path="/World/Ground", z_position=0.0)

FixedCuboid(
    prim_path="/World/Pallet", name="pallet",
    position=np.array([PW / 2, PD / 2, PH / 2]),
    scale=np.array([PW, PD, PH]),
    color=np.array([0.25, 0.35, 0.55]),
)

# --- 2. 스폰 (팔레트 옆에 일렬로) ---
cubes = {}
for i, (name, L, W, H) in enumerate(SPEC):
    cubes[name] = DynamicCuboid(
        prim_path="/World/Boxes/" + name, name=name,
        position=np.array([-1.0, i * 0.55 - 0.5, H / 2 + 0.01]),
        scale=np.array([L, W, H]),
        color=np.array([0.75, 0.55, 0.35]),
        mass=2.0,
    )
print("[1] 스폰 %d 개" % len(SPEC))

# --- 3. 3D 인식 : 바운딩박스로 치수 측정 ---
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
dims = {}
for name, _, _, _ in SPEC:
    prim = stage.GetPrimAtPath("/World/Boxes/" + name)
    s = cache.ComputeWorldBound(prim).ComputeAlignedRange().GetSize()
    dims[name] = (s[0], s[1], s[2])
    print("    %s  L%.3f W%.3f H%.3f" % (name, s[0], s[1], s[2]))
print("[2] 인식 완료")

# --- 4. 적재 : 왼쪽 아래부터 채우고, 줄이 차면 다음 줄 ---
x = y = z = 0.0
row_d = 0.0      # 현재 줄의 최대 깊이
layer_h = 0.0    # 현재 층의 최대 높이
GAP = 0.005
n = 0

for name, _, _, _ in SPEC:
    L, W, H = dims[name]

    if x + L > PW:                 # 줄 바꿈
        x = 0.0
        y += row_d + GAP
        row_d = 0.0
    if y + W > PD:                 # 층 바꿈
        x = y = 0.0
        z += layer_h + GAP
        row_d = layer_h = 0.0

    pos = np.array([x + L / 2, y + W / 2, PH + z + H / 2])
    cubes[name].set_world_pose(position=pos)
    print("    %s -> (%.2f, %.2f, %.2f)" % (name, pos[0], pos[1], pos[2]))

    x += L + GAP
    row_d = max(row_d, W)
    layer_h = max(layer_h, H)
    n += 1

print("[3] 적재 %d 개 완료. Play 를 눌러 확인." % n)