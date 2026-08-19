"""
Pick & Place — 색상 랜덤 큐브 + /rgb 발행 + /color_id 구독

    isaac_python 7_pick_place_color.py

6_pick_place.py 에 다음을 추가한다.
  씬에 있는 큐브 하나를 매 Play 마다
    - 파랑 / 초록 중 랜덤 색상으로 바꾸고
    - 로봇 가동 범위 내 랜덤 위치(pick 지점)로 옮긴다
  Wrist 카메라 이미지를 /rgb(sensor_msgs/Image) 로 발행하고,
  PC B(color_detector)가 보내는 /color_id 를 구독해서, LIFT 이후 큐브를
  든 채로 그 값이 올 때까지 무조건 기다렸다가, 받으면 그 값에 맞는 색
  마커로 Place 한다.
  (ROS 2 가 없거나 /color_id 가 안 오면 WAIT_COLOR 에서 영원히 멈춘다 —
   로컬 fallback 없음, 반드시 PC B 가 /color_id 를 보내야 다음으로 넘어간다)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
from pathlib import Path
import random
import time

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)
from isaacsim.sensors.camera import Camera

# ROS_DOMAIN_ID 는 PC A/PC B 가 반드시 같아야 통신된다. 셸(~/.bashrc 등)에서
# 이미 export 했다면 그 값을 그대로 쓰고, 안 했을 때만 아래 기본값을 쓴다.
# 주의: 이 값이 `ros2 topic list` 를 치는 터미널의 ROS_DOMAIN_ID 와 다르면
# 같은 PC 안에서도 토픽이 안 보인다. RUN 섹션에 실제 값이 찍히니 대조할 것.
os.environ.setdefault("ROS_DOMAIN_ID", "117")

# rclpy 는 시스템 ROS 2(예: /opt/ros/jazzy) 에서 오는 패키지라, Isaac Sim
# 자체 파이썬 환경에는 기본적으로 안 잡힌다. 이 스크립트를 실행한 터미널에서
# ROS 2 를 source 하지 않았으면 여기서 ImportError 가 난다 — 이 경우
# 프로그램이 죽지는 않게 막아두지만, /rgb 발행도 /color_id 수신도 안 되므로
# WAIT_COLOR 단계에서 영원히 멈춘다(의도된 동작 — PC B 없이는 진행 안 됨).
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image as RosImage
    from std_msgs.msg import Int32
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"   WARNING: ROS 2 파이썬 패키지를 못 찾음 ({e})")
    print("   -> 이 터미널에서 ROS 2 를 source 했는지 확인하세요: source /opt/ros/jazzy/setup.bash")
    print("   -> /rgb 발행과 /color_id 수신 둘 다 불가 — WAIT_COLOR 단계에서 계속 멈춰있게 됩니다.")
    ROS_AVAILABLE = False


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH         = str(M0609_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


# ══════════════════════════════════════════════════════════════
#  로봇 설정
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

ROBOT_BASE_POS  = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

# 어깨 높이 / 최대 반경 (URDF 실측, 6_pick_place.py 와 동일)
SHOULDER_Z = 0.1345
SPEC_REACH = 0.900

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  그리퍼 설정
# ══════════════════════════════════════════════════════════════
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.8


# ══════════════════════════════════════════════════════════════
#  TCP 오프셋
# ══════════════════════════════════════════════════════════════
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  큐브 — 색상 / 랜덤 위치
# ══════════════════════════════════════════════════════════════
# 씬에 이미 있는 큐브 prim 을 찾을 때 쓰는 후보 이름.
# 실제 이름이 다르면 이 리스트에 추가하거나, find_cube_prim() 이
# UsdGeom.Cube 타입으로도 자동 탐색하므로 대부분은 그대로 잡힌다.
CUBE_NAME_HINTS = ["blue_block", "cube", "Cube", "target_cube"]

# displayColor 로 우선 시도하고, OmniPBR 등 셰이더가 물려 있으면
# diffuse_color_constant 도 같이 맞춰준다 (둘 다 해줘야 화면에 반영되는 경우가 많다)
COLOR_BLUE  = Gf.Vec3f(0.05, 0.15, 0.9)
COLOR_GREEN = Gf.Vec3f(0.10, 0.75, 0.15)
COLOR_TABLE = {1: ("blue", COLOR_BLUE), 2: ("green", COLOR_GREEN)}

# 큐브가 놓일 표면 높이 (6_pick_place.py 의 PICK_Z 와 동일 기준)
CUBE_Z = 0.05

# 큐브 실제 크기(가로/세로) — USD에서 확인한 xformOp:scale (0.025, 0.025, 0.025)
# 기준. UsdGeom.Cube 기본 size(2.0, 즉 -1~+1) x scale = 0.05m 한 변.
# CUBE_Z 도 0.05 로 이미 맞춰져 있던 값이라 서로 일치한다.
# (모델이 바뀌어 scale 이 달라지면 이 값도 같이 바꿔주면 된다)
CUBE_SCALE     = 0.025
CUBE_BASE_SIZE = 2.0   # UsdGeom.Cube 기본 size 속성
CUBE_SIZE_XY   = (CUBE_SCALE * CUBE_BASE_SIZE, CUBE_SCALE * CUBE_BASE_SIZE)

# 로봇 가동 범위 내에서 pick 위치를 뽑을 극좌표 범위
# base(0,0) 기준, 어깨 앞쪽 부채꼴 영역으로 제한한다.
#   반경: 너무 가까우면 팔이 접혀 IK 가 불안정해지고, 너무 멀면 SPEC_REACH 를 넘는다
#   각도: 로봇 정면 기준 좌우로만 두어 뒤쪽/옆쪽으로 튀지 않게 한다
PICK_RADIUS_MIN   = 0.20
PICK_RADIUS_MAX   = 0.75          # SPEC_REACH(0.900)보다 여유를 둠
PICK_ANGLE_MIN_DEG = -45.0
PICK_ANGLE_MAX_DEG = 45.0

# 놓을 위치 — 색상별 마커와 실제 Place 목적지를 동일하게 맞춘다.
# WAIT_COLOR 단계에서 받은 /color_id(1=파랑/2=초록)로 이 중 하나를 골라
# place_xy 를 확정한다.
PLACE_XY_BLUE  = np.array([0.45, -0.30])
PLACE_XY_GREEN = np.array([0.45,  0.30])
PLACE_XY_TABLE = {1: PLACE_XY_BLUE, 2: PLACE_XY_GREEN}
PLACE_Z = 0.055

# 마커(바닥에 놓는 색상 표시 사각형) 높이 / 기본 크기(실측 실패 시 fallback)
MARKER_Z            = 0.001    # 바닥에서 살짝 띄워 z-fighting 방지
MARKER_SIZE_FALLBACK = 0.12    # 큐브 바운딩박스 계산이 실패할 때만 쓰는 기본값

# 목표 도달 판정 — 이동 단계가 스텝을 다 채워도 실제 TCP 가 목표에서
# 이 거리(m) 이상 떨어져 있으면 도착으로 치지 않고 계속 기다린다.
# (IK 가 간헐적으로 실패해도 팔이 실제로 못 움직였는데 그리퍼가 열려서
#  큐브가 위에서 떨어지는 문제를 막기 위함)
POSITION_TOLERANCE  = 0.01     # 1 cm
MAX_EXTRA_HOLD_STEPS = 120     # 그래도 안 오면 강제 진행(무한 대기 방지)

# ══════════════════════════════════════════════════════════════
#  ROS 2 — /rgb 발행, /color_id 구독
# ══════════════════════════════════════════════════════════════
RGB_TOPIC      = "/rgb"
COLOR_ID_TOPIC = "/color_id"

# Wrist 카메라 prim 을 찾을 때 쓰는 후보 이름. 실제 이름이 다르면 여기 추가.
# (이전 로그에서 확인된 RealSense D455 계열 이름들을 앞에 둔다)
CAMERA_NAME_HINTS = ["RSD455", "realsense_d455", "wrist_camera",
                     "Wrist_Camera", "WristCamera", "camera", "Camera"]
CAMERA_RESOLUTION = (640, 480)

# 매 프레임 발행하면 무거우니 N 스텝마다 한 번만 발행한다.
RGB_PUBLISH_EVERY_STEP = 10

# LIFT 이후 얼마나 기다렸는지 콘솔에 상태 로그를 찍는 주기(스텝).
# 타임아웃은 없다 — /color_id 가 올 때까지 무조건 그 자리에서 기다린다.
WAIT_COLOR_LOG_INTERVAL = 200


# ══════════════════════════════════════════════════════════════
#  보간 파라미터
# ══════════════════════════════════════════════════════════════
APPROACH_HEIGHT = 0.25
LIFT_HEIGHT     = 0.23
GRIPPER_WAIT    = 120

TCP_SPEED  = 0.004
MIN_STEPS  = 60
MAX_STEPS  = 600
HOLD_STEPS = 60

APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG     = 0.0


# ══════════════════════════════════════════════════════════════
#  회전 유틸
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis(axis, deg):
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ══════════════════════════════════════════════════════════════
#  TCP 변환
# ══════════════════════════════════════════════════════════════
def tcp_to_flange(tcp_pos, quat):
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  궤적 보간
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    return start + alpha * (goal - start)


# ══════════════════════════════════════════════════════════════
#  큐브 — prim 탐색 / 색상·위치 랜덤화
# ══════════════════════════════════════════════════════════════
def find_prim_path(root_path, name):
    """USD 계층에서 이름으로 prim 경로를 찾는다 (6_pick_place.py 와 동일)"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def find_cube_prim(root_path="/World"):
    """
    큐브 prim 을 찾는다.
    1순위: 이름이 CUBE_NAME_HINTS 에 있는 prim
    2순위: UsdGeom.Cube 타입 스키마를 쓰는 첫 prim (로봇/카메라 prim 은 자동 제외)
    """
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.GetName() in CUBE_NAME_HINTS:
            return prim

    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Cube):
            return prim

    return None


def find_camera_prim(root_path="/World"):
    """
    Wrist 카메라 prim 을 찾는다. 이름 힌트 우선, 없으면 UsdGeom.Camera
    타입으로 자동 탐색한다. 카메라가 여러 개인 씬이면 이름 힌트로 정확히
    골라야 하므로 CAMERA_NAME_HINTS 를 실제 이름으로 맞춰두는 게 안전하다.
    """
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.GetName() in CAMERA_NAME_HINTS:
            return prim

    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Camera):
            return prim

    return None


def find_renderable_gprims(prim):
    """
    prim 자신과 그 하위(자식) 전체를 훑어서 실제 렌더링 지오메트리(Gprim,
    보통 Mesh)를 전부 모은다. blue_block 같은 이름이 실제로는 빈 Xform
    그룹이고 진짜 메시는 그 자식에 달려 있는 경우가 흔해서, prim 자체만
    보고 색을 바꾸면 아무 변화가 없는 것처럼 보인다.
    """
    gprims = []
    for p in Usd.PrimRange(prim):
        if UsdGeom.Gprim(p):
            gprims.append(p)
    return gprims


def find_material_prims(root_prim):
    """root_prim 하위에서 UsdShade.Material 타입 prim 을 전부 모아 이름->prim 으로 반환한다"""
    materials = {}
    for p in Usd.PrimRange(root_prim):
        if UsdShade.Material(p):
            materials[p.GetName()] = p
    return materials


def set_shader_albedo_tint(material_prim, rgb: Gf.Vec3f):
    """머티리얼에 연결된 셰이더의 Albedo Color Tint(diffuse_tint) 를 바꾼다.
    diffuse_tint 는 텍스처가 있어도 항상 곱해지는 값이라, 텍스처 기반
    머티리얼에서 diffuse_color_constant 만 바꿔선 안 보이던 문제를 우회한다."""
    material = UsdShade.Material(material_prim)
    for shader_out in material.GetSurfaceOutputs():
        source = shader_out.GetConnectedSource()
        if not source:
            continue
        shader = UsdShade.Shader(source[0].GetPrim())
        if not shader:
            continue
        for attr_name in ("diffuse_tint", "diffuse_color_constant", "diffuseColor"):
            shader_input = shader.GetInput(attr_name)
            if shader_input:
                shader_input.Set(rgb)
                print(f"   [color] {material_prim.GetPath()} {attr_name} -> {rgb}")


def set_cube_color(prim, color_name, rgb: Gf.Vec3f):
    """
    prim(큐브) 하위에 있는 Materials 폴더에서 color_name 과 이름이 같은
    머티리얼(Red/Green/Blue/Yellow)을 찾아 그걸로 바인딩을 교체한다.
    같은 이름의 머티리얼이 없으면 현재 바인딩된 머티리얼의 Albedo Tint 만 바꾼다.
    displayColor 도 안전망으로 같이 맞춘다.
    """
    targets = find_renderable_gprims(prim)
    if not targets:
        print(f"   [color] WARNING: {prim.GetPath()} 아래에 Gprim(메시)이 없음 — 색이 안 바뀔 수 있음")
        return

    materials = find_material_prims(prim)
    target_material = next(
        (mat for name, mat in materials.items() if name.lower() == color_name.lower()),
        None,
    )

    for target_prim in targets:
        gprim = UsdGeom.Gprim(target_prim)
        if gprim:
            gprim.GetDisplayColorAttr().Set([rgb])

        binding_api = UsdShade.MaterialBindingAPI(target_prim)

        if target_material is not None:
            binding_api.Bind(UsdShade.Material(target_material))
            print(f"   [color] {target_prim.GetPath()} -> material {target_material.GetPath()}")
        else:
            material, _ = binding_api.ComputeBoundMaterial()
            if material:
                set_shader_albedo_tint(material.GetPrim(), rgb)

    if target_material is not None:
        set_shader_albedo_tint(target_material, rgb)


def sample_pick_xy():
    """로봇 가동 범위(부채꼴) 안에서 랜덤 pick 좌표를 뽑는다"""
    radius = random.uniform(PICK_RADIUS_MIN, PICK_RADIUS_MAX)
    angle_deg = random.uniform(PICK_ANGLE_MIN_DEG, PICK_ANGLE_MAX_DEG)
    angle_rad = np.radians(angle_deg)
    x = radius * np.cos(angle_rad)
    y = radius * np.sin(angle_rad)
    return np.array([x, y])


def apply_random_cube_state(prim):
    """
    큐브의 색상(파랑/초록)과 위치(로봇 가동 범위 내 랜덤)를 새로 뽑아 적용한다.
    반환값: pick_xy (np.array[2]), color_id (1=파랑 / 2=초록), color_name (str)
    """
    color_id = random.choice([1, 2])
    color_name, rgb = COLOR_TABLE[color_id]
    set_cube_color(prim, color_name, rgb)

    pick_xy = sample_pick_xy()
    UsdGeom.XformCommonAPI(prim).SetTranslate(
        Gf.Vec3d(float(pick_xy[0]), float(pick_xy[1]), CUBE_Z)
    )

    return pick_xy, color_id, color_name


def compute_prim_footprint_xy(prim):
    """
    prim(및 하위 전체)의 월드 바운딩박스에서 XY 평면 가로/세로 크기를 구한다.
    큐브가 실제로 회전 없이 정렬돼 있다는 전제(axis-aligned)로, 마커를
    큐브 바닥 크기와 똑같이 맞추는 데 쓴다. 계산 실패 시 (0.0, 0.0) 반환.
    """
    try:
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True,
        )
        bound = bbox_cache.ComputeWorldBound(prim)
        size = bound.ComputeAlignedRange().GetSize()
        return float(size[0]), float(size[1])
    except Exception as e:
        print(f"   [footprint] WARNING: bbox 계산 실패 ({e}) — fallback 크기 사용")
        return 0.0, 0.0


def spawn_flat_marker(stage, path, xy, rgb, size_xy, z=MARKER_Z):
    """
    바닥에 놓는 얇은 색상 사각형 마커를 새로 만든다.
    UsdGeom.Cube 는 기본 한 변이 2(-1~+1) 이므로, scale 로 원하는
    가로세로/두께를 맞추고 displayColor 로 색을 입힌다. 물리 속성은
    붙이지 않아서 로봇/큐브와 충돌하지 않는 순수 시각용 오브젝트다.
    size_xy 는 (가로, 세로) — 큐브 바닥 크기와 동일하게 맞춘다.
    """
    width, depth = size_xy
    if width <= 0.0 or depth <= 0.0:
        width = depth = MARKER_SIZE_FALLBACK

    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(2.0)
    cube.GetDisplayColorAttr().Set([rgb])

    thickness = 0.002
    xform_api = UsdGeom.XformCommonAPI(cube.GetPrim())
    xform_api.SetScale(Gf.Vec3f(width / 2.0, depth / 2.0, thickness))
    xform_api.SetTranslate(Gf.Vec3d(float(xy[0]), float(xy[1]), z))

    print(f"   [marker] {path}  xy={vec(xy)}  size={width:.3f}x{depth:.3f}  color={rgb}")
    return cube.GetPrim()


# ══════════════════════════════════════════════════════════════
#  ROS 2 — /color_id 구독 노드
# ══════════════════════════════════════════════════════════════
if ROS_AVAILABLE:
    class ColorBridgeNode(Node):
        """/rgb(sensor_msgs/Image) 발행 + /color_id(std_msgs/Int32) 구독.
        rclpy.spin_once() 로 메인 루프 안에서 논블로킹 처리한다."""

        def __init__(self):
            super().__init__("m0609_color_pick_place")
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            )
            self._rgb_pub = self.create_publisher(RosImage, RGB_TOPIC, qos)
            self._color_sub = self.create_subscription(
                Int32, COLOR_ID_TOPIC, self._on_color_id, qos
            )
            self.latest_color_id = None
            self.get_logger().info(f"publisher 생성: {RGB_TOPIC} (sensor_msgs/Image)")
            self.get_logger().info(f"subscriber 생성: {COLOR_ID_TOPIC} (std_msgs/Int32)")

        def _on_color_id(self, msg: Int32):
            self.latest_color_id = int(msg.data)
            self.get_logger().info(f"{COLOR_ID_TOPIC} 수신: {self.latest_color_id}")

        def publish_rgb(self, rgba: np.ndarray):
            """카메라의 HxWx4 uint8 배열을 sensor_msgs/Image(rgb8) 로 발행한다"""
            rgb = np.ascontiguousarray(rgba[:, :, :3]).astype(np.uint8)
            msg = RosImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "wrist_camera"
            msg.height, msg.width = int(rgb.shape[0]), int(rgb.shape[1])
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = rgb.tobytes()
            self._rgb_pub.publish(msg)

else:
    class ColorBridgeNode:
        """ROS 2 를 못 쓸 때 쓰는 더미. latest_color_id 는 항상 None이라
        WAIT_COLOR 단계에서 영원히 대기하게 된다 (ROS 2 를 source 하지
        않고 실행하면 이 상태가 된다)."""

        def __init__(self):
            self.latest_color_id = None

        def publish_rgb(self, rgba: np.ndarray):
            pass

        def destroy_node(self):
            pass


# ══════════════════════════════════════════════════════════════
#  Pick & Place 상태 기계
# ══════════════════════════════════════════════════════════════
class PickPlaceFSM:
    """
    0 APPROACH  1 DESCEND  2 GRASP  3 LIFT  4 WAIT_COLOR
    5 MOVE  6 LOWER  7 RELEASE  8 DONE

    pick_xy 는 매 Play 마다 바뀔 수 있으므로 생성자 인자로 받고,
    reset() 을 호출할 때도 새 pick_xy 를 넘기면 waypoint 를 다시 계산한다.

    place_xy 는 생성 시점엔 아직 모르니 임시값(placeholder)만 넣어 둔다.
    WAIT_COLOR 단계에서 실제 /color_id 를 받아야만 그 값에 맞는 마커로
    place_xy 를 계산해서 MOVE/LOWER/RELEASE waypoint 를 채우고 넘어간다.
    받을 때까지는 타임아웃 없이 그 자리에서 계속 기다린다.
    """

    NAMES = ["APPROACH", "DESCEND", "GRASP", "LIFT", "WAIT_COLOR",
              "MOVE", "LOWER", "RELEASE", "DONE"]
    GRIPPER_STATES = {2: "close", 7: "open"}
    WAIT_COLOR_STATE = 4
    DONE_STATE = 8

    def __init__(self, robot, pick_xy, place_xy):
        self._robot = robot
        self.pick_xy = pick_xy
        self.place_xy = place_xy   # WAIT_COLOR 에서 확정되기 전까지의 placeholder
        self.resolved_color_id = None   # WAIT_COLOR 에서 실제로 반영한 color_id (로그/확인용)
        self._build_waypoints()
        self.reset()

    def _build_waypoints(self):
        px, py = self.pick_xy
        gx, gy = self.place_xy
        self.waypoints = [
            np.array([px, py, APPROACH_HEIGHT]),   # 0 APPROACH
            np.array([px, py, CUBE_Z]),             # 1 DESCEND
            np.array([px, py, CUBE_Z]),             # 2 GRASP
            np.array([px, py, LIFT_HEIGHT]),        # 3 LIFT
            np.array([px, py, LIFT_HEIGHT]),        # 4 WAIT_COLOR (제자리 유지)
            np.array([gx, gy, LIFT_HEIGHT]),        # 5 MOVE
            np.array([gx, gy, PLACE_Z]),            # 6 LOWER
            np.array([gx, gy, PLACE_Z]),            # 7 RELEASE
        ]

    def reset(self, pick_xy=None, place_xy=None):
        if pick_xy is not None or place_xy is not None:
            if pick_xy is not None:
                self.pick_xy = pick_xy
            if place_xy is not None:
                self.place_xy = place_xy
            self._build_waypoints()
        self.state = 0
        self.step = 0
        self.start = None
        self.goal = self.waypoints[0]
        self.n_steps = MIN_STEPS
        self.gripper = "open"
        self._extra_hold = 0
        self._wait_color_steps = 0
        self.resolved_color_id = None

    def current_target(self):
        if self.start is None:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self, solved=True, received_color_id=None):
        if self.state >= self.DONE_STATE:
            return

        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]
            self.gripper = self.GRIPPER_STATES.get(self.state, self.gripper)

            if self.state in self.GRIPPER_STATES:
                self.n_steps = GRIPPER_WAIT
                dist = 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)

            print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                  f" goal {vec(self.goal)}"
                  f"  {dist:.4f} m  {self.n_steps} steps  gripper {self.gripper}")

        if self.state == self.WAIT_COLOR_STATE:
            self._advance_wait_color(received_color_id)
            return

        if not solved:
            # IK 가 이번 스텝에 실패했으면 팔이 실제로 못 움직였다는 뜻이라
            # step 을 그대로 두고 같은 목표로 다음 프레임에 다시 시도한다.
            return

        self.step += 1
        if self.step < self.n_steps:
            return

        if self.state not in self.GRIPPER_STATES:
            # 이동 단계는 스텝을 다 채워도 실제 TCP 가 목표 근처까지
            # 왔는지 확인한다. 못 왔으면(간헐적 IK 실패 등) 더 기다렸다가
            # 넘어가서, 팔이 목표 높이에 도달하기 전에 그리퍼가 열려
            # 큐브가 위에서 떨어지는 걸 막는다.
            actual = get_tcp_pose(self._robot)
            error = float(np.linalg.norm(actual - self.goal))
            if error > POSITION_TOLERANCE:
                if self._extra_hold < MAX_EXTRA_HOLD_STEPS:
                    self._extra_hold += 1
                    return
                print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                      f" WARNING 목표 도달 실패 (오차 {error:.4f} m) — 강제 진행")

        self._next()

    def _advance_wait_color(self, received_color_id):
        """/color_id 가 올 때까지 제자리에서 무조건 기다린다(타임아웃 없음).
        받으면 place_xy 를 그 값에 맞는 마커로 계산해서 MOVE 부터의
        waypoint 를 갱신하고 넘어간다."""
        if received_color_id is not None:
            new_place_xy = PLACE_XY_TABLE.get(received_color_id)
            if new_place_xy is None:
                print(f"   [{self.state}] WAIT_COLOR  알 수 없는 color_id={received_color_id} — 무시하고 계속 대기")
            else:
                self.place_xy = new_place_xy
                self.resolved_color_id = received_color_id
                self._build_waypoints()   # 5,6,7 번 waypoint 를 새 place_xy 로 갱신
                print(f"   [{self.state}] WAIT_COLOR  {COLOR_ID_TOPIC}={received_color_id} 수신 -> place {vec(new_place_xy)}")
                self._next()
                return

        self._wait_color_steps += 1
        if self._wait_color_steps % WAIT_COLOR_LOG_INTERVAL == 0:
            print(f"   [{self.state}] WAIT_COLOR  {COLOR_ID_TOPIC} 대기 중... ({self._wait_color_steps} steps)")

    def _next(self):
        self.state += 1
        self.step = 0
        self.start = None
        self._extra_hold = 0
        if self.state >= self.DONE_STATE:
            print(f"   [{self.DONE_STATE}] DONE")


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
class M0609Task(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None
        self._cube_prim = None
        self._cube_footprint = CUBE_SIZE_XY
        self.pick_xy = None
        self.place_xy = None
        self.color_id = None
        self.color_name = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._setup_arm_drives()
        self._register_robot(scene)
        self._find_cube()
        self._spawn_place_markers()
        # world.reset() 이 이 함수 도중에 물리 시뮬레이션을 초기화하므로,
        # 큐브 위치는 반드시 여기서(= reset 이 물리 상태를 굳히기 전에) 정해야 한다.
        # reset 이 끝난 뒤에 옮기면 다음 physics step 에서 원래 위치로 되돌아간다.
        self.randomize_cube()
        print("   scene        ready")

    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()

        print("   USD          loaded")

    def _setup_arm_drives(self):
        stage = omni.usd.get_context().get_stage()
        count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for drive_type in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    count += 1
        print(f"   arm drives   {count}")

    def _register_robot(self, scene):
        ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")

        gripper = ParallelGripper(
            end_effector_prim_path=ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
            joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
            action_deltas=None,
        )

        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=ee_path,
                gripper=gripper,
            )
        )
        print(f"   EE frame     {ee_path}")

    def _find_cube(self):
        self._cube_prim = find_cube_prim("/World")
        if self._cube_prim is None:
            raise RuntimeError(
                "cube prim not found under /World — CUBE_NAME_HINTS 에 실제 이름을 추가하세요"
            )
        # 진단용: 어떤 prim을 찾았는지, 타입이 뭔지, 물리 바디가 붙어있는지,
        # 실제 색을 입힐 하위 Gprim(메시)이 몇 개나 있는지 확인
        has_rigid_body = self._cube_prim.HasAPI(UsdPhysics.RigidBodyAPI)
        gprims = find_renderable_gprims(self._cube_prim)
        materials = find_material_prims(self._cube_prim)
        print(f"   cube prim    {self._cube_prim.GetPath()}")
        print(f"   cube type    {self._cube_prim.GetTypeName()}")
        print(f"   rigid body   {has_rigid_body}")
        print(f"   gprims       {[str(p.GetPath()) for p in gprims]}")
        print(f"   materials    {list(materials.keys())}")

        # 참고용 bbox 계산 — 마커 크기 자체는 이제 CUBE_SIZE_XY(확인된 scale
        # 기반 고정값)로 쓰지만, bbox 값이 크게 다르면 scale 이 바뀌었거나
        # 다른 prim을 찾은 거라는 신호라 로그로만 남겨서 대조한다.
        bbox_footprint = compute_prim_footprint_xy(self._cube_prim)
        print(f"   footprint    fixed={CUBE_SIZE_XY[0]:.3f}x{CUBE_SIZE_XY[1]:.3f}m"
              f"  (bbox 참고값={bbox_footprint[0]:.3f}x{bbox_footprint[1]:.3f}m)")
        self._cube_footprint = CUBE_SIZE_XY

    def _spawn_place_markers(self):
        stage = omni.usd.get_context().get_stage()
        spawn_flat_marker(stage, "/World/place_marker_blue", PLACE_XY_BLUE, COLOR_BLUE, self._cube_footprint)
        spawn_flat_marker(stage, "/World/place_marker_green", PLACE_XY_GREEN, COLOR_GREEN, self._cube_footprint)

    def randomize_cube(self):
        """색상/위치를 새로 뽑아 큐브에 적용한다. place_xy 는 WAIT_COLOR
        단계에서 실제 /color_id 를 받아야 정해지므로, 여기서는 큐브
        자체의 색(같은 이름 마커 좌표)을 placeholder 로만 채워둔다."""
        pick_xy, color_id, color_name = apply_random_cube_state(self._cube_prim)
        self.pick_xy = pick_xy
        self.color_id = color_id
        self.color_name = color_name
        self.place_xy = PLACE_XY_TABLE[color_id]  # WAIT_COLOR 전까지의 placeholder
        return pick_xy, color_id, color_name

    @property
    def robot(self):
        return self._robot

    @property
    def cube_prim(self):
        return self._cube_prim


def init_gripper(robot, world):
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )


def set_ready_pose(robot):
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


# ══════════════════════════════════════════════════════════════
#  IK 솔버
# ══════════════════════════════════════════════════════════════
def create_ik_solver(robot):
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )
    lula.set_robot_base_pose(
        robot_position=ROBOT_BASE_POS,
        robot_orientation=ROBOT_BASE_QUAT,
    )
    print(f"   controlled   {', '.join(lula.get_joint_names())}")
    return ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )


# ══════════════════════════════════════════════════════════════
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def setup_camera(world, camera_prim_path):
    """
    카메라 객체를 (재)생성한다.

    world.reset() 은 물리 시뮬레이션을 통째로 다시 만들기 때문에, 그 전에
    만들어 둔 Camera 객체가 들고 있던 물리 참조가 무효화된다. 그 상태로
    계속 쓰면 rigid body 패턴을 못 찾아 크래시가 나므로, world.reset() 을
    부를 때마다(= Play 를 새로 누를 때마다) 이 함수로 다시 만들어야 한다.

    실패해도 죽지 않고 None 을 반환한다 — /rgb 발행만 꺼지고 나머지는 계속 돈다.
    """
    if camera_prim_path is None:
        return None
    try:
        cam = Camera(prim_path=camera_prim_path, resolution=CAMERA_RESOLUTION)
        cam.initialize()
        for _ in range(5):
            world.step(render=True)   # 렌더 파이프라인 워밍업
        return cam
    except Exception as e:
        print(f"   WARNING: 카메라 초기화 실패 ({e}) — {RGB_TOPIC} 발행 비활성화")
        return None


def print_target_info(target_quat, pick_xy, place_xy, color_name, color_id):
    R = quat_to_matrix(target_quat)
    section("PLAN")
    print(f"   cube color   {color_name} (id={color_id})")
    print(f"   pick xy      {vec(pick_xy)}")
    print(f"   place xy     {vec(place_xy)}  (placeholder — WAIT_COLOR 에서 {COLOR_ID_TOPIC} 받으면 확정)")
    print()
    print(f"   tool   +Z    {vec(R @ np.array([0, 0, 1]))}   approach direction")
    print(f"   finger +X    {vec(R @ np.array([1, 0, 0]))}   finger direction")


def print_status(robot, solved, fsm, target_tcp):
    name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]
    if not solved:
        print(f"   {name:9s} IK FAILED  target {vec(target_tcp)}")
        return
    tcp = get_tcp_pose(robot)
    finger = robot.get_joint_positions()[robot.get_dof_index("finger_joint")]
    print(f"   {name:9s} tcp {vec(tcp)}   finger {finger:+.4f}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    if ROS_AVAILABLE:
        rclpy.init()
    ros_node = ColorBridgeNode()

    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = M0609Task(name="m0609_task")
    world.add_task(task)
    # world.reset() 안에서 set_up_scene() -> randomize_cube() 가 이미 실행되므로
    # 이 시점에는 큐브 색상/위치가 이미 정해져 있다 (물리 초기화 전에 반영됨)
    world.reset()

    robot = task.robot
    robot.initialize()
    init_gripper(robot, world)
    set_ready_pose(robot)
    for _ in range(30):
        world.step(render=True)

    section("CUBE")
    pick_xy, place_xy = task.pick_xy, task.place_xy
    color_id, color_name = task.color_id, task.color_name
    print(f"   color        {color_name} (id={color_id})")
    print(f"   pick xy      {vec(pick_xy)}")
    print(f"   place xy     {vec(place_xy)}  (placeholder)")

    section("CAMERA")
    camera_prim = find_camera_prim("/World")
    camera_prim_path = None
    if camera_prim is None:
        print(f"   WARNING: 카메라 prim을 못 찾음 — CAMERA_NAME_HINTS 에 실제 이름 추가 필요")
        print(f"   -> {RGB_TOPIC} 발행 비활성화")
    else:
        camera_prim_path = str(camera_prim.GetPath())
        print(f"   camera prim  {camera_prim_path}")

    camera = setup_camera(world, camera_prim_path)
    if camera is not None:
        print(f"   {RGB_TOPIC} 발행    {CAMERA_RESOLUTION[0]}x{CAMERA_RESOLUTION[1]}  (매 {RGB_PUBLISH_EVERY_STEP} 스텝)")

    section("SOLVER")
    ik_solver = create_ik_solver(robot)
    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )
    print_target_info(target_quat, pick_xy, place_xy, color_name, color_id)

    section("RUN")
    print(f"   ROS_DOMAIN_ID      = {os.environ.get('ROS_DOMAIN_ID')}")
    print(f"   RMW_IMPLEMENTATION = {os.environ.get('RMW_IMPLEMENTATION', '(unset)')}")
    print(f"   -> `ros2 topic list` 치는 터미널의 두 값과 같아야 토픽이 보입니다")
    print(f"   ROS 2        {'available' if ROS_AVAILABLE else 'NOT available — /color_id 를 영원히 못 받으니 WAIT_COLOR 에서 멈춥니다'}")
    print(f"   {COLOR_ID_TOPIC} 구독 대기 중 (LIFT 이후, 타임아웃 없음 — 받을 때까지 무조건 대기)")
    print("   press Play in the viewport\n")

    fsm = PickPlaceFSM(robot, pick_xy, place_xy)
    was_playing = False
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        if ROS_AVAILABLE:
            rclpy.spin_once(ros_node, timeout_sec=0.0)
        time.sleep(0.005)

        is_playing = world.is_playing()

        if is_playing and not was_playing:
            # 반드시 world.reset() '이전에' 랜덤화한다 — reset 이 그 순간의
            # USD 상태로 물리 시뮬레이션을 초기화하기 때문에, reset 뒤에 위치를
            # 바꾸면 다음 physics step 에서 물리엔진이 원래 위치로 되돌려 놓는다.
            pick_xy, color_id, color_name = task.randomize_cube()
            place_xy = task.place_xy  # WAIT_COLOR 에서 실제 /color_id 로 확정될 placeholder

            world.reset()
            robot.initialize()
            init_gripper(robot, world)
            set_ready_pose(robot)
            # world.reset() 으로 물리 시뮬레이션이 통째로 다시 만들어졌으므로
            # 이전 camera 객체는 무효화됐다 — 반드시 다시 만들어야 한다.
            camera = setup_camera(world, camera_prim_path)
            fsm.reset(pick_xy=pick_xy, place_xy=place_xy)
            ros_node.latest_color_id = None  # 이전 사이클 값이 새 사이클로 새지 않게 초기화
            step = 0
            print()
            section("CUBE")
            print(f"   color        {color_name} (id={color_id})")
            print(f"   pick xy      {vec(pick_xy)}")
            print(f"   place xy     {vec(place_xy)}  (placeholder)")

        if is_playing:
            target_tcp = fsm.current_target()
            flange_target = tcp_to_flange(target_tcp, target_quat)

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )
            if solved:
                robot.apply_action(action)

            robot.apply_action(robot.gripper.forward(action=fsm.gripper))

            if camera is not None and step % RGB_PUBLISH_EVERY_STEP == 0:
                try:
                    rgba = camera.get_rgba()
                    if rgba is not None and rgba.size > 0:
                        ros_node.publish_rgb(rgba)
                except Exception as e:
                    print(f"   WARNING: {RGB_TOPIC} 캡처/발행 실패 ({e}) — 카메라 비활성화")
                    camera = None

            fsm.advance(solved=solved, received_color_id=ros_node.latest_color_id)

            if step % LOG_INTERVAL == 0:
                print_status(robot, solved, fsm, target_tcp)
            step += 1

        was_playing = is_playing

    ros_node.destroy_node()
    if ROS_AVAILABLE:
        rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()