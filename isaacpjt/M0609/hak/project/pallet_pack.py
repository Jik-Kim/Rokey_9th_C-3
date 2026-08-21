"""
1차 목표 파이프라인 : 택배 상자 스폰 -> 3D 인식(치수) -> 팔레트 적재

Isaac Sim Script Editor 에서:
    exec(open("/home/rokey/pallet_pack.py").read())

패커만 따로 테스트 (일반 python):
    python3 pallet_pack.py
"""

import random

# ----------------------------------------------------------------------
# 설정
# ----------------------------------------------------------------------
SEED = 42

PALLET_W = 1.10          # T11 팔레트 (m)
PALLET_D = 1.10
PALLET_H = 0.15          # 데크 두께
MAX_STACK_H = 1.20       # 팔레트 상면 기준 최대 적재고

SUPPORT_MIN = 0.70       # 지지면적 비율 하한
GAP = 0.005              # 박스 간 클리어런스 (붐 처짐 대비)

N_BOXES = 40             # 스폰할 박스 수
MEAS_NOISE = 0.000       # 인식 오차 시뮬 (m). 0.003 정도 넣으면 강건성 테스트

#        name    L      W      H      m_min  m_max  share%
BOX_TYPES = [
    ("std_1", 0.220, 0.190, 0.090,  0.3,  1.5, 15),
    ("std_2", 0.270, 0.180, 0.150,  0.5,  3.0, 25),
    ("std_3", 0.340, 0.250, 0.210,  1.0,  6.0, 30),
    ("std_4", 0.410, 0.310, 0.280,  2.0, 12.0, 20),
    ("std_5", 0.480, 0.380, 0.340,  3.0, 15.0, 10),
]

COLORS = {
    "std_1": (0.85, 0.75, 0.55),
    "std_2": (0.80, 0.65, 0.45),
    "std_3": (0.75, 0.55, 0.35),
    "std_4": (0.65, 0.45, 0.28),
    "std_5": (0.55, 0.35, 0.22),
}


# ----------------------------------------------------------------------
# 1. 박스 목록 생성 (스폰 대상)
# ----------------------------------------------------------------------
def make_manifest(n, rng):
    names = [t[0] for t in BOX_TYPES]
    weights = [t[6] for t in BOX_TYPES]
    spec = {t[0]: t for t in BOX_TYPES}
    out = []
    for i in range(n):
        nm = rng.choices(names, weights=weights, k=1)[0]
        _, L, W, H, mmin, mmax, _ = spec[nm]
        out.append({
            "id": "box_%03d" % i,
            "type": nm,
            "dims": (L, W, H),
            "mass": round(rng.uniform(mmin, mmax), 2),
        })
    return out


# ----------------------------------------------------------------------
# 2. 3D 인식 (치수만)
#    지금은 USD 바운딩박스에서 읽는 스텁.
#    나중에 Replicator bounding_box_3d 어노테이터로 이 함수만 교체하면 됨.
# ----------------------------------------------------------------------
def measure_from_usd(prim, rng):
    from pxr import Usd, UsdGeom
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng_box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    s = rng_box.GetSize()
    d = [s[0], s[1], s[2]]
    if MEAS_NOISE > 0:
        d = [max(0.01, v + rng.gauss(0, MEAS_NOISE)) for v in d]
    return (d[0], d[1], d[2])


# ----------------------------------------------------------------------
# 3. 패커 : DBLF + 지지면적 제약
# ----------------------------------------------------------------------
def _overlap_1d(a0, a1, b0, b1):
    return max(0.0, min(a1, b1) - max(a0, b0))


EPS = 1e-6


def _collides(placed, x, y, z, l, w, h, gap):
    # 각 축을 EPS 만큼 줄여서 '맞닿음'은 충돌로 보지 않는다.
    # (특히 z: 아래 박스 윗면과 접촉하는 것을 충돌로 오판하면 2단이 안 쌓인다)
    x0, x1 = x - gap + EPS, x + l + gap - EPS
    y0, y1 = y - gap + EPS, y + w + gap - EPS
    z0, z1 = z + EPS, z + h - EPS
    for p in placed:
        if _overlap_1d(x0, x1, p["x"], p["x"] + p["l"]) <= EPS:
            continue
        if _overlap_1d(y0, y1, p["y"], p["y"] + p["w"]) <= EPS:
            continue
        if _overlap_1d(z0, z1, p["z"], p["z"] + p["h"]) <= EPS:
            continue
        return True
    return False


def _support_ratio(placed, x, y, z, l, w):
    if z <= 1e-6:
        return 1.0
    need = l * w
    got = 0.0
    for p in placed:
        if abs((p["z"] + p["h"]) - z) > 1e-3:
            continue
        ox = _overlap_1d(x, x + l, p["x"], p["x"] + p["l"])
        oy = _overlap_1d(y, y + w, p["y"], p["y"] + p["w"])
        got += ox * oy
    return got / need if need > 0 else 0.0


def _load_ok(placed, x, y, z, l, w, mass):
    """아래 박스 허용하중 = 3 kPa x 밑면적. 값 미정이라 가정치."""
    if z <= 1e-6:
        return True
    for p in placed:
        if abs((p["z"] + p["h"]) - z) > 1e-3:
            continue
        ox = _overlap_1d(x, x + l, p["x"], p["x"] + p["l"])
        oy = _overlap_1d(y, y + w, p["y"], p["y"] + p["w"])
        if ox * oy <= 0:
            continue
        cap = 3000.0 * (p["l"] * p["w"]) / 9.81      # kg
        if p["carried"] + mass > cap:
            return False
    return True


def _commit_load(placed, x, y, z, l, w, mass):
    for p in placed:
        if abs((p["z"] + p["h"]) - z) > 1e-3:
            continue
        ox = _overlap_1d(x, x + l, p["x"], p["x"] + p["l"])
        oy = _overlap_1d(y, y + w, p["y"], p["y"] + p["w"])
        if ox * oy > 0:
            p["carried"] += mass


def pack(items, W=PALLET_W, D=PALLET_D, H=MAX_STACK_H,
         support=SUPPORT_MIN, gap=GAP, order=None):
    """items: [{"id","dims"(l,w,h),"mass"}]  ->  (placed, leftover)"""
    placed = []
    pts = [(0.0, 0.0, 0.0)]

    if order is None:
        order = sorted(range(len(items)),
                       key=lambda i: -(items[i]["dims"][0] *
                                       items[i]["dims"][1] *
                                       items[i]["dims"][2]))
    leftover = []

    for i in order:
        it = items[i]
        l0, w0, h = it["dims"]
        m = it["mass"]
        best = None
        for (px, py, pz) in pts:
            for (bl, bw, yaw) in ((l0, w0, 0.0), (w0, l0, 90.0)):
                if px + bl > W + 1e-9 or py + bw > D + 1e-9 or pz + h > H + 1e-9:
                    continue
                if _collides(placed, px, py, pz, bl, bw, h, gap):
                    continue
                if _support_ratio(placed, px, py, pz, bl, bw) < support - 1e-9:
                    continue
                if not _load_ok(placed, px, py, pz, bl, bw, m):
                    continue
                key = (round(pz, 4), round(py, 4), round(px, 4))
                if best is None or key < best[0]:
                    best = (key, px, py, pz, bl, bw, yaw)
        if best is None:
            leftover.append(it["id"])
            continue

        _, px, py, pz, bl, bw, yaw = best
        _commit_load(placed, px, py, pz, bl, bw, m)
        placed.append({"id": it["id"], "type": it.get("type", "?"),
                       "x": px, "y": py, "z": pz,
                       "l": bl, "w": bw, "h": h,
                       "yaw": yaw, "mass": m, "carried": 0.0})
        for np_ in ((px + bl + gap, py, pz),
                    (px, py + bw + gap, pz),
                    (px, py, pz + h)):
            if np_ not in pts:
                pts.append(np_)

    return placed, leftover


# ----------------------------------------------------------------------
# 4. 리포트
# ----------------------------------------------------------------------
def report(placed, leftover, n_total):
    if not placed:
        print("[!] 배치된 박스가 없습니다.")
        return
    top = max(p["z"] + p["h"] for p in placed)
    vol = sum(p["l"] * p["w"] * p["h"] for p in placed)
    env = PALLET_W * PALLET_D * top
    mass = sum(p["mass"] for p in placed)
    cx = sum((p["x"] + p["l"] / 2) * p["mass"] for p in placed) / mass
    cy = sum((p["y"] + p["w"] / 2) * p["mass"] for p in placed) / mass
    cz = sum((p["z"] + p["h"] / 2) * p["mass"] for p in placed) / mass

    print("-" * 46)
    print(" 배치 %d / %d   (미배치 %d)" % (len(placed), n_total, len(leftover)))
    print(" 적재고      %.3f m" % top)
    print(" 체적 적재율 %.1f %%  (박스 %.4f / 외포 %.4f m3)" % (100 * vol / env, vol, env))
    print(" 총 질량     %.1f kg" % mass)
    print(" 무게중심    x %.3f  y %.3f  z %.3f" % (cx, cy, cz))
    print(" CoG 편차    x %+.3f  y %+.3f  (중심 대비, m)" % (cx - PALLET_W / 2, cy - PALLET_D / 2))
    print("-" * 46)


# ----------------------------------------------------------------------
# 5. Isaac Sim 씬 구축
# ----------------------------------------------------------------------
def build_in_isaac(manifest):
    import numpy as np
    import omni.usd
    try:
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, GroundPlane
    except ImportError:
        from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane

    rng = random.Random(SEED)
    stage = omni.usd.get_context().get_stage()

    GroundPlane(prim_path="/World/Ground", z_position=0.0)

    # 팔레트 (고정)
    FixedCuboid(
        prim_path="/World/Pallet", name="pallet",
        position=np.array([PALLET_W / 2, PALLET_D / 2, PALLET_H / 2]),
        scale=np.array([PALLET_W, PALLET_D, PALLET_H]),
        color=np.array([0.25, 0.35, 0.55]),
    )

    # --- 스폰 : 팔레트 옆 스테이징 라인 ---
    prims = {}
    for k, it in enumerate(manifest):
        L, W, H = it["dims"]
        row, col = divmod(k, 10)
        pos = np.array([-1.2 - row * 0.6, col * 0.55 - 0.5, H / 2 + 0.001])
        c = COLORS[it["type"]]
        cube = DynamicCuboid(
            prim_path="/World/Boxes/" + it["id"], name=it["id"],
            position=pos, scale=np.array([L, W, H]),
            color=np.array(c), mass=it["mass"],
        )
        prims[it["id"]] = cube
    print("[1] 스폰 완료 : %d 개" % len(manifest))

    # --- 3D 인식 : 스폰된 프림에서 치수 측정 ---
    for it in manifest:
        prim = stage.GetPrimAtPath("/World/Boxes/" + it["id"])
        it["dims_meas"] = measure_from_usd(prim, rng)
    print("[2] 인식 완료 : 치수 측정")
    for it in manifest[:3]:
        print("    %s %s  spec %.3f/%.3f/%.3f  meas %.3f/%.3f/%.3f"
              % ((it["id"], it["type"]) + it["dims"] + it["dims_meas"]))

    # --- 패킹 : 측정값으로 계획 ---
    items = [{"id": it["id"], "type": it["type"],
              "dims": it["dims_meas"], "mass": it["mass"]} for it in manifest]
    placed, leftover = pack(items)
    print("[3] 패킹 완료")

    # --- 배치 : 계산 좌표로 텔레포트 ---
    for p in placed:
        cube = prims[p["id"]]
        pos = np.array([p["x"] + p["l"] / 2,
                        p["y"] + p["w"] / 2,
                        PALLET_H + p["z"] + p["h"] / 2])
        if abs(p["yaw"] - 90.0) < 1e-6:
            q = np.array([0.70710678, 0.0, 0.0, 0.70710678])   # w,x,y,z
            cube.set_world_pose(position=pos, orientation=q)
        else:
            cube.set_world_pose(position=pos)
    print("[4] 배치 완료 : Play 를 눌러 안정성 확인")

    report(placed, leftover, len(manifest))
    return placed, leftover


# ----------------------------------------------------------------------
def main():
    rng = random.Random(SEED)
    manifest = make_manifest(N_BOXES, rng)
    try:
        import omni.usd  # noqa: F401
        in_isaac = True
    except ImportError:
        in_isaac = False

    if in_isaac:
        build_in_isaac(manifest)
    else:
        print("[Isaac 밖] 패커만 실행합니다.")
        items = [{"id": it["id"], "type": it["type"],
                  "dims": it["dims"], "mass": it["mass"]} for it in manifest]
        placed, leftover = pack(items)
        report(placed, leftover, len(manifest))


main()