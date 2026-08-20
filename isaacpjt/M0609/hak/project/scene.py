"""
씬 구성 — 컨베이어 / 팔레트 / 박스 스폰 / 스캔 카메라

1차 목표는 한 라인이므로 2차 컨베이어 분기(A/B/C)와 트럭은 만들지 않는다.
컨베이어 1개 + 팔레트 1개 + 로봇 1대.
"""

from __future__ import annotations

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import config as C


# ─────────────────────────────────────────────────────────────
# 재질
# ─────────────────────────────────────────────────────────────
def _material(stage, path: str, rgb, roughness: float = 0.6):
    UsdGeom.Scope.Define(stage, "/World/Looks")
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _bind(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


# ─────────────────────────────────────────────────────────────
# 정적 구조물
# ─────────────────────────────────────────────────────────────
def _static_box(stage, path: str, center, size, rgb):
    """움직이지 않는 충돌체 상자. center 는 상자 중심."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])

    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in center)))
    xform.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in size)))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    _bind(cube.GetPrim(), _material(stage, f"/World/Looks/{path.split('/')[-1]}Mat", rgb))
    return cube


def build_conveyor(stage):
    """
    1차 컨베이어. 지금은 이송 구동 없이 박스가 올라오는 평면 역할만 한다.
    CONVEYOR_CENTER 의 z 는 벨트 하단 높이이고 상면은 CONVEYOR_TOP_Z 다.
    """
    center = C.CONVEYOR_CENTER + np.array([0.0, 0.0, C.CONVEYOR_SIZE[2] / 2.0])
    conveyor = _static_box(stage, C.CONVEYOR_PATH, center, C.CONVEYOR_SIZE,
                           (0.22, 0.24, 0.28))
    # 벨트가 공중에 뜨지 않도록 다리를 세운다 (시각용)
    if C.CONVEYOR_CENTER[2] > 0.05:
        leg_h = C.CONVEYOR_CENTER[2]
        for i, sx in enumerate((-0.45, 0.45)):
            for j, sy in enumerate((-0.4, 0.4)):
                _static_box(
                    stage, f"{C.CONVEYOR_PATH}_Leg{i}{j}",
                    np.array([C.CONVEYOR_CENTER[0] + sx * C.CONVEYOR_SIZE[0],
                              C.CONVEYOR_CENTER[1] + sy * C.CONVEYOR_SIZE[1],
                              leg_h / 2.0]),
                    np.array([0.05, 0.05, leg_h]), (0.25, 0.27, 0.30))
    return conveyor


def build_pedestal(stage):
    """
    로봇 받침대. 실제 팔레타이징 셀은 로봇을 올려 세운다.

    바닥 직결이면 만재 상단(1040mm)에 팔이 닿지 않는다. 받침대 높이가
    그대로 어깨 높이로 더해져 도달 범위가 위로 확장된다.
    """
    if C.PEDESTAL_H <= 0.0:
        return None
    size = np.array([0.50, 0.50, C.PEDESTAL_H])
    center = np.array([float(C.ROBOT_BASE_XY[0]), float(C.ROBOT_BASE_XY[1]),
                       C.PEDESTAL_H / 2.0])
    return _static_box(stage, C.PEDESTAL_PATH, center, size, (0.30, 0.32, 0.36))


def build_pallet(stage):
    """
    팔레트를 만든다.

    PALLET_USD 가 있으면 Isaac Sim 기본 자산(pallet.usd = EUR1 유로 팔레트,
    실측 1213 x 802 x 142.5mm)을 참조하고, 없으면 소형 팔레트를 직접 만든다.
    어느 쪽이든 상판 윗면이 정확히 PALLET_DECK_Z 에 오도록 맞춘다.
    """
    if getattr(C, "PALLET_USD", None):
        pallet = _pallet_from_usd(stage)
    else:
        pallet = _pallet_procedural(stage)

    _pallet_border(stage)
    return pallet


def _pallet_from_usd(stage):
    """Isaac 기본 팔레트 자산을 참조한다."""
    from isaacsim.core.utils.stage import add_reference_to_stage

    add_reference_to_stage(usd_path=C.PALLET_USD, prim_path=C.PALLET_PATH)
    prim = stage.GetPrimAtPath(C.PALLET_PATH)

    # 자산은 바닥(z=0)에 밑면이 놓이도록 만들어져 있다. 상판 두께가
    # PALLET_DECK_Z 와 다르면 그 차이만큼 z 를 보정한다.
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = np.array(rng.GetMin()), np.array(rng.GetMax())
    ex, ey = float(hi[0] - lo[0]), float(hi[1] - lo[1])
    cx, cy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0

    # Isaac 의 pallet.usd 는 긴 변이 X 축이다. 우리 설계는 긴 변을 Y(접선방향)에
    # 두므로, 방향이 다르면 Z 축으로 90도 돌린다. 이걸 빼먹으면 팔레트가 90도
    # 틀어진 채 놓여 배치 지점이 팔레트 밖으로 나가고 박스가 바닥으로 떨어진다.
    want_long_x = float(C.PALLET_SIZE[0]) >= float(C.PALLET_SIZE[1])
    has_long_x = ex >= ey
    rotate = want_long_x != has_long_x

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()

    if rotate:
        # Z 축 +90도: (x, y) -> (-y, x)
        rcx, rcy = -cy, cx
    else:
        rcx, rcy = cx, cy

    xform.AddTranslateOp().Set(Gf.Vec3d(
        float(C.PALLET_CENTER_XY[0] - rcx),
        float(C.PALLET_CENTER_XY[1] - rcy),
        float(C.PALLET_DECK_Z - hi[2]),
    ))
    if rotate:
        half = np.radians(90.0) / 2.0
        xform.AddOrientOp().Set(
            Gf.Quatf(float(np.cos(half)), Gf.Vec3f(0.0, 0.0, float(np.sin(half))))
        )
    return prim


def _pallet_procedural(stage):
    """
    소형 팔레트(EUR6 하프 등)를 판재 구조로 만든다.

    Isaac Sim 에는 소형 팔레트 자산이 없다. EUR1 을 비균등 스케일하면
    판재 두께까지 찌그러지므로, 실제 팔레트 구조(상판 / 각목 / 하판)를
    그대로 쌓아 만든다.
    """
    W, D = float(C.PALLET_SIZE[0]), float(C.PALLET_SIZE[1])
    H = float(C.PALLET_DECK_Z)

    top_t, bot_t = 0.022, 0.022          # 상/하판 두께
    block_h = H - top_t - bot_t          # 각목 높이
    wood = (0.62, 0.47, 0.29)

    UsdGeom.Scope.Define(stage, C.PALLET_PATH)
    cx, cy = float(C.PALLET_CENTER_XY[0]), float(C.PALLET_CENTER_XY[1])

    # 상판 — D 방향으로 5장
    n_top, gap = 5, 0.02
    board_d = (D - (n_top - 1) * gap) / n_top
    for i in range(n_top):
        y = cy - D / 2.0 + board_d / 2.0 + i * (board_d + gap)
        _static_box(stage, f"{C.PALLET_PATH}/Top{i}",
                    np.array([cx, y, H - top_t / 2.0]),
                    np.array([W, board_d, top_t]), wood)

    # 각목 — 3 x 3
    blk = 0.10
    for i, fx in enumerate((-1, 0, 1)):
        for j, fy in enumerate((-1, 0, 1)):
            _static_box(stage, f"{C.PALLET_PATH}/Block{i}{j}",
                        np.array([cx + fx * (W / 2.0 - blk / 2.0),
                                  cy + fy * (D / 2.0 - blk / 2.0),
                                  bot_t + block_h / 2.0]),
                        np.array([blk, blk, block_h]), (0.55, 0.41, 0.25))

    # 하판 — D 방향 3장
    for i, fy in enumerate((-1, 0, 1)):
        y = cy + fy * (D / 2.0 - 0.10)
        _static_box(stage, f"{C.PALLET_PATH}/Bot{i}",
                    np.array([cx, y, bot_t / 2.0]),
                    np.array([W, 0.10, bot_t]), wood)

    return stage.GetPrimAtPath(C.PALLET_PATH)


def _pallet_border(stage):
    """적재 결과를 눈으로 보기 위한 상판 경계 표시."""
    border = UsdGeom.Scope.Define(stage, f"{C.PALLET_PATH}_Border")
    t = 0.006
    W, D = float(C.PALLET_SIZE[0]), float(C.PALLET_SIZE[1])
    edges = [
        (np.array([0.0, D / 2.0]), np.array([W, t])),
        (np.array([0.0, -D / 2.0]), np.array([W, t])),
        (np.array([W / 2.0, 0.0]), np.array([t, D])),
        (np.array([-W / 2.0, 0.0]), np.array([t, D])),
    ]
    for i, (offset, extent) in enumerate(edges):
        path = f"{C.PALLET_PATH}_Border/Edge{i}"
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        xform = UsdGeom.Xformable(cube)
        xform.AddTranslateOp().Set(Gf.Vec3d(
            float(C.PALLET_CENTER_XY[0] + offset[0]),
            float(C.PALLET_CENTER_XY[1] + offset[1]),
            float(C.PALLET_DECK_Z + 0.002),
        ))
        xform.AddScaleOp().Set(Gf.Vec3f(float(extent[0]), float(extent[1]), 0.003))
        cube.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.85, 0.1)])
    return border


# ─────────────────────────────────────────────────────────────
# 박스
# ─────────────────────────────────────────────────────────────
BOX_COLORS = [
    (0.80, 0.58, 0.34),
    (0.72, 0.50, 0.28),
    (0.86, 0.66, 0.42),
    (0.66, 0.46, 0.26),
]


def spawn_box(stage, index: int, rng: np.random.Generator):
    """
    컨베이어 픽 존에 박스 하나를 만든다.

    Returns: (prim_path, size(3,), yaw_deg, mass_kg, 규격이름)
    """
    UsdGeom.Scope.Define(stage, C.BOX_ROOT)
    path = f"{C.BOX_ROOT}/Box_{index:03d}"

    if C.BOX_MODE == "spec":
        # 혼입 비율(3호 50% / 4호 30% / 5호 20%)대로 뽑는다
        weights = np.array([C.BOX_RATIO[n] for n in C.BOX_NAMES], dtype=float)
        weights /= weights.sum()
        spec_name = C.BOX_NAMES[int(rng.choice(len(C.BOX_NAMES), p=weights))]
        size = C.BOX_SPECS[spec_name].copy()
    else:
        size = rng.uniform(C.BOX_MIN, C.BOX_MAX, size=3)
        spec_name = "random"
    yaw = float(rng.uniform(*C.SPAWN_YAW_RANGE_DEG))

    jitter = rng.uniform(-C.SPAWN_JITTER_XY, C.SPAWN_JITTER_XY, size=2)
    pos = np.array([
        C.PICK_XY[0] + jitter[0],
        C.PICK_XY[1] + jitter[1],
        C.CONVEYOR_TOP_Z + size[2] / 2.0 + 0.001,
    ])

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])

    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in pos)))
    xform.AddOrientOp().Set(_yaw_quat(yaw))
    xform.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in size)))

    prim = cube.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)

    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass = (float(C.BOX_MASS[spec_name]) if C.BOX_MODE == "spec"
            else float(np.prod(size) * C.BOX_DENSITY))
    mass_api.CreateMassAttr().Set(mass)

    rgb = BOX_COLORS[index % len(BOX_COLORS)]
    _bind(prim, _material(stage, f"/World/Looks/BoxMat_{index % len(BOX_COLORS)}", rgb))

    return path, size, yaw, mass, spec_name


def _yaw_quat(yaw_deg: float) -> Gf.Quatf:
    """AddOrientOp() 는 기본이 float precision 이므로 Quatf 여야 한다."""
    half = np.radians(yaw_deg) / 2.0
    return Gf.Quatf(float(np.cos(half)), Gf.Vec3f(0.0, 0.0, float(np.sin(half))))


def clear_boxes(stage):
    root = stage.GetPrimAtPath(C.BOX_ROOT)
    if root.IsValid():
        stage.RemovePrim(C.BOX_ROOT)


# ─────────────────────────────────────────────────────────────
# 스캔 카메라 (PERCEPTION_MODE = "camera")
# ─────────────────────────────────────────────────────────────
def build_scan_camera(stage):
    """픽 존을 수직으로 내려다보는 카메라. USD 카메라는 -Z 를 본다."""
    camera = UsdGeom.Camera.Define(stage, C.CAM_PATH)
    camera.CreateFocalLengthAttr(float(C.CAM_FOCAL_LENGTH))
    camera.CreateHorizontalApertureAttr(float(C.CAM_HORIZONTAL_APERTURE))
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))

    xform = UsdGeom.Xformable(camera)
    xform.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in C.CAM_POS)))
    # 기본 자세로 이미 -Z(아래)를 보므로 회전 없음
    xform.AddOrientOp().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))

    return camera


def lock_scan_camera(stage):
    """
    스캔 카메라를 설정된 위치/자세로 되돌린다.

    뷰포트를 이 카메라 시점으로 두고 마우스로 움직이면 Isaac Sim 이
    카메라 프림 자체를 옮겨버린다. 실제 라인의 고정 스캐너는 움직이지
    않으므로, 매 스캔 전에 원위치로 강제한다.
    """
    prim = stage.GetPrimAtPath(C.CAM_PATH)
    if not prim.IsValid():
        return False

    xform = UsdGeom.Xformable(prim)
    pos = Gf.Vec3d(*(float(v) for v in C.CAM_POS))
    rot = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))   # 기본 자세 = -Z(아래)를 봄

    translate_op = orient_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            orient_op = op

    # 뷰포트 조작이 op 구성 자체를 바꿔놨으면 처음부터 다시 만든다
    if translate_op is None or orient_op is None:
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(pos)
        xform.AddOrientOp().Set(rot)
        return True

    if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
        translate_op.Set(pos)
    else:
        translate_op.Set(Gf.Vec3f(*(float(v) for v in C.CAM_POS)))

    if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
        orient_op.Set(Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0)))
    else:
        orient_op.Set(rot)
    return True


def build_lights(stage):
    from pxr import UsdLux

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(900.0)

    key = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr(2200.0)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 12.0, 0.0))
    return dome, key
