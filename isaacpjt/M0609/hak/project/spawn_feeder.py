"""공장 투입구 피더 — Play 하면 박스가 하나씩 벨트 위로 떨어진다 (총 10개).

Isaac Sim GUI 에서 돌린다.

    ./run_spawn_feeder.sh                       # test1.usd 를 열고 피더를 붙인다
    PL_N=20 PL_INTERVAL=2 ./run_spawn_feeder.sh

이미 스테이지를 열어 둔 상태라면 Script Editor 에 이 파일 내용을 붙여넣어도 된다
(현재 열린 스테이지가 test1.usd 면 다시 열지 않는다).

동작
  · Play 중 물리 스텝에서 시간을 누적해 PL_INTERVAL 초마다 박스 1개 생성.
  · 호수는 3/4/5호 를 50:30:20 비율로 랜덤, 요(yaw)와 가로 위치도 랜덤.
    가로 지터는 회전 후 실제 발자국으로 상한을 계산해 실측한 벨트 폭 밖으로
    걸치지 않게 한다 (투입 벨트는 1001mm).
  · 색은 호수 고정 — 3호 빨강 / 4호 초록 / 5호 파랑.
    test1.usd 에 이미 있는 /World/Looks/BoxNo{3,4,5} 머티리얼을 쓰고,
    없으면 만든다. displayColor 도 같이 넣는다.
  · Stop 하면 스폰한 박스를 전부 지우고 카운터를 리셋한다. 다시 Play 하면 처음부터.

투입 위치 (PL_SPAWN)
  스테이지에서 벨트 메시 bbox 를 재서 폭 중심 · 상면 · 진입단을 그때그때 구한다.
  씬에서 벨트를 옮기거나 높이를 바꿔도 따라간다.
  cube   : 공장 투입구 (/World/ConveyorTrack, 기본). x=0 쪽에서 들어와 -X 로
           흐른다. 진입단에서 0.5m 안쪽에 떨어뜨린다.
             투입 -> -X 로 주행 -> x=-8 A03 커브 -> 북상 (본선 x=-9.50)
             -> SorterBlue -> SorterGreen -> SorterRed -> ConveyorTrack_01
  feeder : 본선 시작점 (/World/ConveyorTrack_03). +Y 로 흐른다. 커브를 건너뛰고
           바로 북상시켜 소터 구간만 볼 때 쓴다.

낙하 높이 (PL_DROP_H, 기본 0.30m)
  실측한 벨트 상면 기준이다. 0.30m 면 착지 2.4m/s 라 9kg 5호도 안 튄다.
  2026-08-21 실측: 라인 전 구간이 1780mm 계열이다 (A06 벨트 1780.5mm,
  소터 롤러 1756~1759mm, ConveyorTrack_04 롤러 1737.5mm). 예전 900mm 평탄화
  버전이 아니다. 그래서 상면을 상수로 박지 않고 매번 잰다.
"""

import math
import os
import random

import omni.timeline
import omni.usd
from omni.physx import get_physx_interface
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

STAGE_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

N_BOXES = int(os.environ.get("PL_N", 10))
INTERVAL = float(os.environ.get("PL_INTERVAL", 3.0))   # 박스 사이 간격 [초]
SEED = os.environ.get("PL_SEED")                       # 없으면 매번 다른 랜덤
SPAWN = os.environ.get("PL_SPAWN", "cube")
DROP_H = float(os.environ.get("PL_DROP_H", "0.30"))   # 벨트면 위 낙하 높이 [m]

BOX_ROOT = "/World/Boxes"
LOOKS_ROOT = "/World/Looks"
CUBE_PATH = "/World/Cube"

BOX_SPECS = {
    "3호": (0.340, 0.250, 0.210),
    "4호": (0.410, 0.310, 0.280),
    "5호": (0.480, 0.360, 0.340),
}
BOX_MASS = {"3호": 3.0, "4호": 5.0, "5호": 9.0}
BOX_RATIO = {"3호": 0.50, "4호": 0.30, "5호": 0.20}
BOX_COLOR = {"3호": (1.0, 0.0, 0.0), "4호": (0.0, 1.0, 0.0), "5호": (0.0, 0.0, 1.0)}
BOX_SLUG = {"3호": "No3", "4호": "No4", "5호": "No5"}

# 투입 지점은 상수로 박지 않는다. 스테이지에서 벨트 메시를 직접 재서 구한다.
# 예전엔 벨트면 z=0.900, 폭 중심 (-0.478, 0.035) 로 박아 뒀는데 씬을 1780mm
# 높이 계열로 되돌린 뒤로 그 좌표가 벨트면보다 470mm 아래를 가리켰다. 박스가
# 벨트 안쪽에서 생성돼 튕겨 나가거나 바닥으로 떨어졌다 — 스폰이 안 되던 원인.
#
#   이름: (조각 프림, (흐름 축, 부호), 진입단에서 안쪽으로 넣을 거리 [m])
SPAWN_POINTS = {
    # 공장 투입구. ConveyorTrack 은 x=0 쪽으로 들어와 -X 로 흐른다.
    "cube":   ("/World/ConveyorTrack",    ("x", -1), 0.50),
    # 로봇까지 이어지는 본선 시작점. ConveyorTrack_03 은 +Y 로 흐른다.
    "feeder": ("/World/ConveyorTrack_03", ("y", +1), 0.40),
}

# 박스가 닿는 면의 하위 프림 이름. 에셋 계열마다 다르다 —
# A05/A02/A43 (770mm 계열) 은 Rollers, A06/A03 (1780mm 계열) 은 Belt.
DECK_NAMES = ("Rollers", "Belt")

# 실측이 안 될 때만 쓰는 값 (2026-08-21 헤드리스 실측 기록).
FALLBACK = {
    "cube":   {"top": 1.7805, "center": (-0.500, 0.000), "half": 0.5005},
    "feeder": {"top": 1.7805, "center": (-9.496, 1.896), "half": 0.4500},
}

BELT_TOP_Z = FALLBACK["cube"]["top"]    # start() 에서 실측값으로 덮어쓴다
BELT_HALF_W = FALLBACK["cube"]["half"]
EDGE_MARGIN = 0.08       # 벨트 가장자리 여유 [m]
YAW_JITTER_DEG = 15.0

_state = {}


# ─────────────────────────────────────────────────────────────
def start():
    stop()

    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if stage is None or stage.GetRootLayer().identifier != STAGE_PATH:
        print(f"[feeder] 스테이지 열기: {STAGE_PATH}")
        ctx.open_stage(STAGE_PATH)
        stage = ctx.get_stage()

    center, axis = _measure(stage)
    drop_z = BELT_TOP_Z + DROP_H

    # 투입구에 있던 테스트 큐브는 비활성화한다 (스폰 박스와 부딪히므로).
    # 파일을 고치는 게 아니라 세션에서만 꺼지고, Ctrl+S 하면 저장된다.
    cube = stage.GetPrimAtPath(CUBE_PATH)
    if cube and cube.IsActive():
        cube.SetActive(False)
        print(f"[feeder] {CUBE_PATH} 비활성화 (되살리려면 Property 창에서 active 체크)")

    _ensure_materials(stage)
    if not stage.GetPrimAtPath(BOX_ROOT):
        UsdGeom.Xform.Define(stage, BOX_ROOT)

    _state.update(
        stage=stage, center=center, axis=axis, drop_z=drop_z,
        rng=random.Random(int(SEED)) if SEED is not None else random.Random(),
        elapsed=INTERVAL, spawned=0, paths=[],
    )

    _state["physics_sub"] = get_physx_interface().subscribe_physics_step_events(_on_step)
    _state["timeline_sub"] = (
        omni.timeline.get_timeline_interface()
        .get_timeline_event_stream()
        .create_subscription_to_pop(_on_timeline)
    )

    print(f"[feeder] 투입구 '{SPAWN}' ({center[0]:.3f}, {center[1]:.3f}, {drop_z:.3f})  "
          f"낙하 {DROP_H:.2f}m -> 착지 {math.sqrt(2 * 9.81 * DROP_H):.1f}m/s")
    print(f"[feeder] Play 를 누르면 {INTERVAL:.1f}초마다 1개씩, 총 {N_BOXES}개 투입한다.")


def stop():
    """구독 해제 + 스폰한 박스 제거."""
    if _state.get("physics_sub") is not None:
        _state["physics_sub"] = None
    if _state.get("timeline_sub") is not None:
        _state["timeline_sub"] = None
    _clear()
    _state.clear()


# ─────────────────────────────────────────────────────────────
def _on_step(dt):
    if _state.get("spawned", 0) >= N_BOXES:
        return
    _state["elapsed"] += dt
    if _state["elapsed"] < INTERVAL:
        return
    _state["elapsed"] = 0.0
    _spawn_one()


def _on_timeline(event):
    if event.type == int(omni.timeline.TimelineEventType.STOP):
        # 되감기: 스폰한 박스를 지우고 처음부터 다시 시작할 수 있게 한다
        _clear()
        _state["spawned"] = 0
        _state["elapsed"] = INTERVAL
        print("[feeder] Stop — 투입 박스 제거, 카운터 리셋")


def _clear():
    stage = _state.get("stage")
    if stage is None:
        return
    for path in _state.get("paths", []):
        if stage.GetPrimAtPath(path):
            stage.RemovePrim(path)
    _state["paths"] = []


# ─────────────────────────────────────────────────────────────
def _measure(stage):
    """투입 벨트 메시를 재서 (중심 xy, 폭 방향 축) 을 돌려준다.

    같이 BELT_TOP_Z (벨트 상면) 와 BELT_HALF_W (반폭) 를 실측값으로 채운다.
    bbox 는 purpose 를 전부 넣고 잡는다 — default 만 넣으면 proxy 로 잡힌
    충돌 메시가 빠져서 빈 범위가 나온다.
    """
    global BELT_TOP_Z, BELT_HALF_W
    if SPAWN not in SPAWN_POINTS:
        raise ValueError(f"PL_SPAWN 은 {' | '.join(SPAWN_POINTS)}: {SPAWN!r}")

    track_path, (flow_axis, flow_sign), inset = SPAWN_POINTS[SPAWN]
    cross_axis = "y" if flow_axis == "x" else "x"
    fb = FALLBACK[SPAWN]

    prim, prim_path = None, f"{track_path}/<{'|'.join(DECK_NAMES)}>"
    for deck in DECK_NAMES:
        p = stage.GetPrimAtPath(f"{track_path}/{deck}")
        if p and p.IsValid():
            prim, prim_path = p, f"{track_path}/{deck}"
            break

    rng = None
    if prim is not None:
        try:
            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                 UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide])
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not r.IsEmpty():
                rng = (r.GetMin(), r.GetMax())
        except Exception as exc:
            print(f"[feeder] 벨트 측정 실패 ({exc})")
    if rng is None:
        print(f"[feeder] {prim_path} 를 못 쟀다 — 기록해 둔 실측값을 쓴다")
        BELT_TOP_Z, BELT_HALF_W = fb["top"], fb["half"]
        return fb["center"], cross_axis

    mn, mx = rng
    i_flow = 0 if flow_axis == "x" else 1
    i_cross = 1 - i_flow
    # 진입단 = 흐름의 반대쪽 끝. 거기서 inset 만큼 안쪽에 떨어뜨린다.
    entry = mx[i_flow] if flow_sign < 0 else mn[i_flow]
    along = float(entry + flow_sign * inset)
    cross_c = float((mn[i_cross] + mx[i_cross]) / 2.0)

    BELT_TOP_Z = float(mx[2])
    BELT_HALF_W = float((mx[i_cross] - mn[i_cross]) / 2.0)
    center = (along, cross_c) if flow_axis == "x" else (cross_c, along)

    print(f"[feeder] 벨트 실측 {prim_path}")
    print(f"[feeder]   상면 {BELT_TOP_Z * 1000:.0f}mm  폭 {BELT_HALF_W * 2 * 1000:.0f}mm  "
          f"중심 ({center[0]:.3f}, {center[1]:.3f})")
    return center, cross_axis


def _spawn_one():
    stage, rng = _state["stage"], _state["rng"]
    cx, cy = _state["center"]
    axis = _state["axis"]

    name = rng.choices(list(BOX_SPECS), weights=[BOX_RATIO[k] for k in BOX_SPECS])[0]
    dims = BOX_SPECS[name]

    yaw = (90.0 if rng.random() < 0.5 else 0.0) + rng.uniform(-YAW_JITTER_DEG, YAW_JITTER_DEG)
    c, s = abs(math.cos(math.radians(yaw))), abs(math.sin(math.radians(yaw)))
    # 벨트를 가로지르는 방향의 반폭 (진행축에 따라 x/y 가 바뀐다)
    half = (dims[0] * c + dims[1] * s) / 2.0 if axis == "x" else (dims[0] * s + dims[1] * c) / 2.0
    jitter = max(0.0, BELT_HALF_W - EDGE_MARGIN - half)
    off = rng.uniform(-jitter, jitter)

    # (cx, cy) 는 실측한 벨트 폭 중심이므로 off 만 걸면 항상 벨트 안이다.
    x = cx + off if axis == "x" else cx
    y = cy if axis == "x" else cy + off
    z = _state["drop_z"] + dims[2] / 2.0

    i = _state["spawned"]
    path = f"{BOX_ROOT}/Box_{i:02d}_{BOX_SLUG[name]}"
    _make_box(stage, path, name, dims, (x, y, z), yaw)

    _state["paths"].append(path)
    _state["spawned"] = i + 1
    print(f"[feeder] {i + 1:2d}/{N_BOXES}  {name}  "
          f"{dims[0] * 1000:.0f}x{dims[1] * 1000:.0f}x{dims[2] * 1000:.0f}mm  "
          f"{BOX_MASS[name]:.0f}kg  yaw={yaw:+6.1f}도  pos=({x:.3f}, {y:.3f}, {z:.3f})")


def _make_box(stage, path, name, dims, pos, yaw_deg):
    cube = UsdGeom.Cube.Define(stage, path)
    prim = cube.GetPrim()
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    cube.CreateDisplayColorAttr([Gf.Vec3f(*BOX_COLOR[name])])

    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    half = math.radians(yaw_deg) / 2.0
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half))))
    xf.AddScaleOp().Set(Gf.Vec3d(*dims))

    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxCollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(BOX_MASS[name])


    mat = UsdShade.Material(stage.GetPrimAtPath(f"{LOOKS_ROOT}/Box{BOX_SLUG[name]}"))
    if mat:
        UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(mat)


def _ensure_materials(stage):
    if not stage.GetPrimAtPath(LOOKS_ROOT):
        UsdGeom.Scope.Define(stage, LOOKS_ROOT)
    for name, rgb in BOX_COLOR.items():
        mat_path = f"{LOOKS_ROOT}/Box{BOX_SLUG[name]}"
        if stage.GetPrimAtPath(mat_path):
            continue
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        print(f"[feeder] 머티리얼 생성 {mat_path}")


start()
