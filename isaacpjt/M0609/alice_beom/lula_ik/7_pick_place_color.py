"""
PC A 최종본 — Isaac Sim Standalone Pick & Place + ROS 2

- Wrist Camera -> /rgb (sensor_msgs/msg/Image)
- /color_id (std_msgs/msg/Int32) 구독
- 1 -> BLUE place / 2 -> GREEN place
- Play 시작마다 Pick cube 색상 + WORLD X/Y 위치 랜덤 설정
- color_id가 늦으면 LIFT 위치에서 대기
"""

import os
os.environ["ROS_DOMAIN_ID"] = "117"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import random
import time
import numpy as np
import omni.usd

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleRigidPrim, SingleXFormPrim
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)

enable_extension("isaacsim.ros2.bridge")
for _ in range(10):
    simulation_app.update()

import rclpy
from std_msgs.msg import Int32


# ─────────────────────────────────────────────────────────────
# 경로
# ─────────────────────────────────────────────────────────────
THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH = str(M0609_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")
URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


# ─────────────────────────────────────────────────────────────
# 로봇 / 그리퍼 / TCP
# ─────────────────────────────────────────────────────────────
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME = "link_6"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8

ROBOT_BASE_POS = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSE_POS = 0.8

FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ─────────────────────────────────────────────────────────────
# Pick / Place
# ─────────────────────────────────────────────────────────────
PICK_XY = np.array([0.25, 0.10])  # 기본/초기 Pick 위치

# Pick Cube 랜덤 Spawn 영역 (meter)
# 필요하면 아래 4개 값만 수정하면 됨.
SPAWN_X_MIN = 0.18
SPAWN_X_MAX = 0.42
SPAWN_Y_MIN = -0.22
SPAWN_Y_MAX = 0.22

# 실제 Stage의 마커 좌표가 다르면 이 두 줄만 수정
BLUE_PLACE_XY = np.array([0.45, -0.30])
GREEN_PLACE_XY = np.array([0.45, 0.30])

PLACE_TARGETS = {
    1: BLUE_PLACE_XY,
    2: GREEN_PLACE_XY,
}

PLACE_XY = BLUE_PLACE_XY.copy()

PICK_Z = 0.05
PLACE_Z = 0.055
APPROACH_HEIGHT = 0.25
LIFT_HEIGHT = 0.23

GRIPPER_WAIT = 120
TCP_SPEED = 0.004
MIN_STEPS = 60
MAX_STEPS = 600

APPROACH_ROLL_DEG = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG = 0.0


# ─────────────────────────────────────────────────────────────
# ROS 2 / Camera
# ─────────────────────────────────────────────────────────────
ROS_DOMAIN_ID = 117
RGB_TOPIC = "/rgb"
COLOR_TOPIC = "/color_id"
CAMERA_FRAME_ID = "wrist_camera"
CAMERA_RESOLUTION = (640, 480)

COLOR_ACCEPT_DELAY_STEPS = 15

CUBE_COLORS = {
    1: (0.02, 0.12, 1.00),   # BLUE
    2: (0.02, 0.80, 0.12),   # GREEN
}


# ─────────────────────────────────────────────────────────────
# Place 바닥 마커
# ─────────────────────────────────────────────────────────────
MARKER_SIZE = 0.06       # 12 cm x 12 cm
MARKER_THICKNESS = 0.003  # 3 mm
MARKER_Z = MARKER_THICKNESS / 2.0

BLUE_MARKER_PATH = "/World/PlaceMarkers/BluePlaceMarker"
GREEN_MARKER_PATH = "/World/PlaceMarkers/GreenPlaceMarker"
BLUE_MARKER_MATERIAL_PATH = "/World/Looks/BluePlaceMarkerMaterial"
GREEN_MARKER_MATERIAL_PATH = "/World/Looks/GreenPlaceMarkerMaterial"


# ─────────────────────────────────────────────────────────────
# 회전 / TCP
# ─────────────────────────────────────────────────────────────
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_from_axis(axis, deg):
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a /= np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    q = quat_mul(
        quat_from_axis([1, 0, 0], roll_deg),
        quat_from_axis([0, 1, 0], pitch_deg),
    )
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def tcp_to_flange(tcp_pos, quat):
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


def steps_for(start, goal):
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    return start + alpha * (goal - start)


# ─────────────────────────────────────────────────────────────
# Pick & Place FSM
# ─────────────────────────────────────────────────────────────
class PickPlaceFSM:
    NAMES = ["APPROACH", "DESCEND", "GRASP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "DONE"]
    GRIPPER_STATES = {2: "close", 6: "open"}
    MOVE_STATE = 4
    DONE_STATE = 7

    def __init__(self, robot):
        self._robot = robot
        self._build_waypoints()
        self.reset()

    def _build_waypoints(self):
        px, py = PICK_XY
        gx, gy = PLACE_XY
        self.waypoints = [
            np.array([px, py, APPROACH_HEIGHT]),
            np.array([px, py, PICK_Z]),
            np.array([px, py, PICK_Z]),
            np.array([px, py, LIFT_HEIGHT]),
            np.array([gx, gy, LIFT_HEIGHT]),
            np.array([gx, gy, PLACE_Z]),
            np.array([gx, gy, PLACE_Z]),
        ]

    def set_pick(self, pick_xy):
        """랜덤 Spawn된 Cube 위치에 맞춰 Pick waypoint를 갱신."""
        px, py = float(pick_xy[0]), float(pick_xy[1])

        self.waypoints[0] = np.array([px, py, APPROACH_HEIGHT])
        self.waypoints[1] = np.array([px, py, PICK_Z])
        self.waypoints[2] = np.array([px, py, PICK_Z])
        self.waypoints[3] = np.array([px, py, LIFT_HEIGHT])

        # reset() 직후 첫 APPROACH 목표도 새 위치로 변경
        if self.state == 0:
            self.goal = self.waypoints[0]
            self.start = None
            self.step = 0

    def set_place(self, place_xy):
        gx, gy = place_xy
        self.waypoints[4] = np.array([gx, gy, LIFT_HEIGHT])
        self.waypoints[5] = np.array([gx, gy, PLACE_Z])
        self.waypoints[6] = np.array([gx, gy, PLACE_Z])

    def reset(self):
        self.state = 0
        self.step = 0
        self.start = None
        self.goal = self.waypoints[0]
        self.n_steps = MIN_STEPS
        self.gripper = "open"

    def current_target(self):
        if self.start is None:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
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

            print(
                f"   [{self.state}] {self.NAMES[self.state]:9s}"
                f" goal {vec(self.goal)}"
                f"  {dist:.4f} m  {self.n_steps} steps"
                f"  gripper {self.gripper}"
            )

        self.step += 1
        if self.step >= self.n_steps:
            self._next()

    def _next(self):
        self.state += 1
        self.step = 0
        self.start = None

        if self.state >= self.DONE_STATE:
            print(f"   [{self.DONE_STATE}] DONE")


# ─────────────────────────────────────────────────────────────
# Scene
# ─────────────────────────────────────────────────────────────
def find_prim_path(root_path, name):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)

    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())

    return None


def find_wrist_camera_path():
    stage = omni.usd.get_context().get_stage()

    root = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if root.IsValid():
        cameras = [p for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Camera)]
        if cameras:
            cameras.sort(
                key=lambda p: (
                    "wrist" not in str(p.GetPath()).lower(),
                    "camera" not in p.GetName().lower(),
                )
            )
            return str(cameras[0].GetPath())

    cameras = [p for p in stage.Traverse() if p.IsA(UsdGeom.Camera)]
    return str(cameras[0].GetPath()) if cameras else None


def _find_rigid_body_ancestor(prim):
    """
    visual Cube/Mesh가 RigidBody Xform의 자식일 수 있으므로
    실제로 움직여야 하는 physics body prim을 위쪽에서 찾는다.
    """
    current = prim

    while current.IsValid():
        try:
            if current.HasAPI(UsdPhysics.RigidBodyAPI):
                return current
        except Exception:
            pass

        parent = current.GetParent()
        if not parent.IsValid() or str(parent.GetPath()) in ("/", "/World"):
            break

        current = parent

    return None


def find_pick_cube_prims():
    """
    반환:
        visual_prim : 실제 보이는 Pick cube geometry prim
        body_prim   : 랜덤 Spawn 때 이동할 RigidBody root
                      RigidBody가 없으면 visual_prim 자체

    선택 원칙:
      1) UsdGeom.Cube를 최우선
      2) 이름/path에 'cube'가 있는 Mesh/Gprim
      3) 마지막 fallback으로 이름에 'cube'가 있는 Xform 계열
    """
    stage = omni.usd.get_context().get_stage()

    geometry_candidates = []
    fallback_candidates = []

    for prim in stage.Traverse():
        path = str(prim.GetPath())

        if path.startswith(ROBOT_PRIM_PATH + "/"):
            continue

        if path.startswith("/World/PlaceMarkers/"):
            continue

        if path.startswith("/World/Looks/"):
            continue

        name = prim.GetName().lower()
        path_lower = path.lower()

        try:
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            p = matrix.ExtractTranslation()
            xy = np.array([float(p[0]), float(p[1])])
            dist = float(np.linalg.norm(xy - PICK_XY))
        except Exception:
            dist = 9999.0

        # 실제 geometry를 우선
        if prim.IsA(UsdGeom.Cube):
            geometry_candidates.append((0, dist, prim))
            continue

        if prim.IsA(UsdGeom.Mesh) and (
            "cube" in name or "cube" in path_lower
        ):
            geometry_candidates.append((1, dist, prim))
            continue

        # fallback: 이름만 cube인 Xform 등
        if "cube" in name:
            fallback_candidates.append((dist, prim))

    if geometry_candidates:
        geometry_candidates.sort(key=lambda item: (item[0], item[1]))
        visual_prim = geometry_candidates[0][2]
    elif fallback_candidates:
        fallback_candidates.sort(key=lambda item: item[0])
        visual_prim = fallback_candidates[0][1]
    else:
        return None, None

    rigid_prim = _find_rigid_body_ancestor(visual_prim)
    body_prim = rigid_prim if rigid_prim is not None else visual_prim

    print(f"   cube visual  {visual_prim.GetPath()}")
    print(f"   cube body    {body_prim.GetPath()}")

    return visual_prim, body_prim


def set_cube_color(cube_prim, color_id):
    if cube_prim is None or not cube_prim.IsValid():
        raise RuntimeError("Pick cube prim을 찾지 못했습니다.")

    stage = omni.usd.get_context().get_stage()
    rgb = CUBE_COLORS[color_id]

    UsdGeom.Scope.Define(stage, "/World/Looks")

    material = UsdShade.Material.Define(
        stage,
        "/World/Looks/RandomPickCubeMaterial",
    )

    shader = UsdShade.Shader.Define(
        stage,
        "/World/Looks/RandomPickCubeMaterial/Shader",
    )

    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput(
        "diffuseColor",
        Sdf.ValueTypeNames.Color3f,
    ).Set(Gf.Vec3f(*rgb))
    shader.CreateInput(
        "roughness",
        Sdf.ValueTypeNames.Float,
    ).Set(0.35)
    shader.CreateInput(
        "metallic",
        Sdf.ValueTypeNames.Float,
    ).Set(0.0)

    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(),
        "surface",
    )

    UsdShade.MaterialBindingAPI.Apply(cube_prim).Bind(material)

    name = "BLUE" if color_id == 1 else "GREEN"
    print(f"   cube         {cube_prim.GetPath()}")
    print(f"   random color {name}")


def _get_usd_world_position(prim):
    """USD hierarchy 기준 prim의 world translation을 읽는다."""
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    p = matrix.ExtractTranslation()
    return np.array(
        [float(p[0]), float(p[1]), float(p[2])],
        dtype=float,
    )


def set_cube_spawn_position(
    world,
    cube_visual_prim,
    cube_body_prim,
    spawn_xy,
):
    """
    요청한 WORLD XY에 '보이는 Cube 중심'이 오도록 이동한다.

    중요:
    - RigidBody root와 보이는 Cube geometry의 pivot/중심은 다를 수 있다.
    - 따라서 body root를 spawn_xy에 바로 두면 안 된다.
    - 이동 전 visual-body WORLD offset을 계산하고,
      그 offset을 보상한 body target을 사용한다.
    - FSM에는 body root가 아니라 보이는 Cube의 실제/예측 WORLD XY를 반환한다.
    """
    if cube_visual_prim is None or not cube_visual_prim.IsValid():
        raise RuntimeError("Pick cube visual prim을 찾지 못했습니다.")

    if cube_body_prim is None or not cube_body_prim.IsValid():
        raise RuntimeError("Pick cube body prim을 찾지 못했습니다.")

    requested_visual_xy = np.array(
        [float(spawn_xy[0]), float(spawn_xy[1])],
        dtype=float,
    )

    body_path = str(cube_body_prim.GetPath())

    is_rigid = False
    try:
        is_rigid = cube_body_prim.HasAPI(UsdPhysics.RigidBodyAPI)
    except Exception:
        is_rigid = False

    if is_rigid:
        cube_handle = SingleRigidPrim(
            prim_path=body_path,
            name="pick_cube_rigid_handle",
            reset_xform_properties=False,
        )
        cube_handle.initialize(
            physics_sim_view=world.physics_sim_view
        )

        body_before, body_orientation = cube_handle.get_world_pose()

        # reset 직후의 authored hierarchy에서 보이는 cube 중심을 읽는다.
        visual_before = _get_usd_world_position(cube_visual_prim)

        # 보이는 geometry 중심과 physics body root 사이의 WORLD offset.
        # orientation은 spawn 중 유지하므로 이 offset도 그대로 유지된다.
        visual_minus_body = visual_before - np.asarray(
            body_before,
            dtype=float,
        )

        # visual center가 requested_visual_xy에 오도록 body root를 보정 이동
        target_body_position = np.array(
            [
                requested_visual_xy[0] - visual_minus_body[0],
                requested_visual_xy[1] - visual_minus_body[1],
                float(body_before[2]),
            ],
            dtype=float,
        )

        cube_handle.set_world_pose(
            position=target_body_position,
            orientation=body_orientation,
        )

        # 이전 trial 속도 제거
        cube_handle.set_linear_velocity(
            np.zeros(3, dtype=float)
        )
        cube_handle.set_angular_velocity(
            np.zeros(3, dtype=float)
        )

        body_after, _ = cube_handle.get_world_pose()
        body_after = np.asarray(body_after, dtype=float)

        # orientation을 바꾸지 않았으므로 같은 world offset을 적용 가능
        visual_after = body_after + visual_minus_body

    else:
        # RigidBody가 없는 단순 Xform/Cube
        cube_handle = SingleXFormPrim(
            prim_path=body_path,
            name="pick_cube_xform_handle",
            reset_xform_properties=False,
        )

        current_position, current_orientation = cube_handle.get_world_pose()

        target_position = np.array(
            [
                requested_visual_xy[0],
                requested_visual_xy[1],
                float(current_position[2]),
            ],
            dtype=float,
        )

        cube_handle.set_world_pose(
            position=target_position,
            orientation=current_orientation,
        )

        actual_position, _ = cube_handle.get_world_pose()
        visual_after = np.asarray(actual_position, dtype=float)

    actual_visual_xy = np.array(
        [
            float(visual_after[0]),
            float(visual_after[1]),
        ],
        dtype=float,
    )

    error_xy = float(
        np.linalg.norm(
            actual_visual_xy - requested_visual_xy
        )
    )

    print(
        f"   spawn request VISUAL "
        f"({requested_visual_xy[0]:.4f}, {requested_visual_xy[1]:.4f})"
    )

    if is_rigid:
        print(
            f"   visual-body offset "
            f"({visual_minus_body[0]:+.4f}, "
            f"{visual_minus_body[1]:+.4f})"
        )

    print(
        f"   spawn actual VISUAL "
        f"({actual_visual_xy[0]:.4f}, {actual_visual_xy[1]:.4f})"
    )
    print(
        f"   spawn XY error "
        f"{error_xy * 1000.0:.2f} mm"
    )

    if error_xy > 0.002:
        print(
            "   WARNING      visual spawn error > 2 mm"
        )

    # FSM/그리퍼가 실제 보이는 cube 중심 XY를 사용
    return actual_visual_xy


def _ensure_scope(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        UsdGeom.Scope.Define(stage, path)


def create_marker_material(stage, material_path, color):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")

    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput(
        "diffuseColor",
        Sdf.ValueTypeNames.Color3f,
    ).Set(Gf.Vec3f(*color))
    shader.CreateInput(
        "roughness",
        Sdf.ValueTypeNames.Float,
    ).Set(0.4)
    shader.CreateInput(
        "metallic",
        Sdf.ValueTypeNames.Float,
    ).Set(0.0)

    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(),
        "surface",
    )
    return material


def create_floor_marker(stage, prim_path, xy, color, material_path):
    # 이전 실행에서 같은 마커가 남아 있으면 새로 만들기 위해 제거
    old_prim = stage.GetPrimAtPath(prim_path)
    if old_prim.IsValid():
        stage.RemovePrim(prim_path)

    marker = UsdGeom.Cube.Define(stage, prim_path)
    marker.CreateSizeAttr(1.0)

    xform = UsdGeom.Xformable(marker.GetPrim())
    xform.AddTranslateOp().Set(
        Gf.Vec3d(
            float(xy[0]),
            float(xy[1]),
            float(MARKER_Z),
        )
    )
    xform.AddScaleOp().Set(
        Gf.Vec3f(
            float(MARKER_SIZE),
            float(MARKER_SIZE),
            float(MARKER_THICKNESS),
        )
    )

    material = create_marker_material(
        stage,
        material_path,
        color,
    )

    UsdShade.MaterialBindingAPI.Apply(
        marker.GetPrim()
    ).Bind(material)

    return marker.GetPrim()


def create_place_markers():
    stage = omni.usd.get_context().get_stage()

    # 물리 충돌이 없는 시각용 마커만 생성
    if not stage.GetPrimAtPath("/World/PlaceMarkers").IsValid():
        UsdGeom.Xform.Define(stage, "/World/PlaceMarkers")

    _ensure_scope(stage, "/World/Looks")

    create_floor_marker(
        stage=stage,
        prim_path=BLUE_MARKER_PATH,
        xy=BLUE_PLACE_XY,
        color=CUBE_COLORS[1],
        material_path=BLUE_MARKER_MATERIAL_PATH,
    )

    create_floor_marker(
        stage=stage,
        prim_path=GREEN_MARKER_PATH,
        xy=GREEN_PLACE_XY,
        color=CUBE_COLORS[2],
        material_path=GREEN_MARKER_MATERIAL_PATH,
    )

    print(f"   blue marker  {vec(BLUE_PLACE_XY)}")
    print(f"   green marker {vec(GREEN_PLACE_XY)}")


class M0609Task(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._setup_arm_drives()
        self._register_robot(scene)
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
            raise RuntimeError(
                f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}"
            )

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

    @property
    def robot(self):
        return self._robot


# ─────────────────────────────────────────────────────────────
# ROS 2
# ─────────────────────────────────────────────────────────────
class ColorReceiver:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node("isaac_color_receiver")
        self.color_id = None
        self.message_count = 0

        self.subscription = self.node.create_subscription(
            Int32,
            COLOR_TOPIC,
            self._callback,
            1,
        )

        print(f"   subscribe    {COLOR_TOPIC}  std_msgs/msg/Int32")

    def _callback(self, msg):
        value = int(msg.data)

        if value not in PLACE_TARGETS:
            print(f"   ROS2         invalid color_id={value}")
            return

        self.color_id = value
        self.message_count += 1

        name = "BLUE" if value == 1 else "GREEN"
        print(f"   ROS2 RX      color_id={value} ({name})")


# ─────────────────────────────────────────────────────────────
# Robot init / IK
# ─────────────────────────────────────────────────────────────
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


# ─────────────────────────────────────────────────────────────
# 출력
# ─────────────────────────────────────────────────────────────
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def print_status(robot, solved, fsm, target_tcp):
    name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]

    if not solved:
        print(f"   {name:9s} IK FAILED  target {vec(target_tcp)}")
        return

    tcp = get_tcp_pose(robot)
    finger = robot.get_joint_positions()[
        robot.get_dof_index("finger_joint")
    ]

    print(
        f"   {name:9s}"
        f" tcp {vec(tcp)}"
        f" finger {finger:+.4f}"
    )


# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
LOG_INTERVAL = 60


def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")

    task = M0609Task(name="m0609_task")
    world.add_task(task)
    world.reset()

    # 파랑/초록 Place 바닥 마커 생성
    create_place_markers()

    robot = task.robot
    robot.initialize()
    init_gripper(robot, world)
    set_ready_pose(robot)

    for _ in range(30):
        world.step(render=True)

    camera_path = find_wrist_camera_path()
    if camera_path is None:
        raise RuntimeError("Wrist Camera를 찾지 못했습니다.")

    cube_visual_prim, cube_body_prim = find_pick_cube_prims()

    if cube_visual_prim is None or cube_body_prim is None:
        raise RuntimeError("Pick cube를 찾지 못했습니다.")

    section("ROS2")

    color_receiver = ColorReceiver()

    section("SOLVER")

    ik_solver = create_ik_solver(robot)

    target_quat = make_target_quat(
        APPROACH_ROLL_DEG,
        APPROACH_PITCH_DEG,
        GRIPPER_YAW_DEG,
    )

    section("PLAN")
    print(f"   default pick {vec(PICK_XY)}")
    print(
        f"   spawn range  WORLD X[{SPAWN_X_MIN:.2f}, {SPAWN_X_MAX:.2f}] "
        f"Y[{SPAWN_Y_MIN:.2f}, {SPAWN_Y_MAX:.2f}]"
    )
    print(f"   blue  (1)    {vec(BLUE_PLACE_XY)}")
    print(f"   green (2)    {vec(GREEN_PLACE_XY)}")
    print(f"   marker size  {MARKER_SIZE:.3f} m square")
    print(f"   rgb topic    {RGB_TOPIC}  (existing USD Camera Helper)")
    print(f"   color topic  {COLOR_TOPIC}")
    print(f"   domain id    {ROS_DOMAIN_ID}")

    section("RUN")
    print("   press Play in the viewport")
    print("   color_id: 1=BLUE, 2=GREEN")
    print("   color_id가 늦으면 LIFT 위치에서 기다립니다.\n")

    fsm = PickPlaceFSM(robot)

    was_playing = False
    step = 0

    accepted_color_id = None
    run_start_message_count = 0

    while simulation_app.is_running():
        world.step(render=True)

        rclpy.spin_once(
            color_receiver.node,
            timeout_sec=0.0,
        )

        time.sleep(0.005)
        is_playing = world.is_playing()

        # Play 시작
        if is_playing and not was_playing:
            world.reset()

            robot.initialize()
            init_gripper(robot, world)
            set_ready_pose(robot)

            fsm.reset()
            step = 0

            # ─────────────────────────────────────────────
            # Pick Cube 위치 랜덤 Spawn
            # ─────────────────────────────────────────────
            random_pick_xy = np.array([
                random.uniform(SPAWN_X_MIN, SPAWN_X_MAX),
                random.uniform(SPAWN_Y_MIN, SPAWN_Y_MAX),
            ])

            # 실제 Cube BODY를 WORLD 좌표로 이동
            # 그리고 보이는 Cube 중심의 실제 WORLD XY를 반환받음
            actual_pick_xy = set_cube_spawn_position(
                world,
                cube_visual_prim,
                cube_body_prim,
                random_pick_xy,
            )

            # gripper/FSM은 요청 랜덤값이 아니라
            # 실제 Cube WORLD 위치를 그대로 사용
            fsm.set_pick(
                actual_pick_xy,
            )

            # ─────────────────────────────────────────────
            # Pick Cube 색상 랜덤
            # ─────────────────────────────────────────────
            random_color_id = random.choice([1, 2])
            set_cube_color(
                cube_visual_prim,
                random_color_id,
            )

            # 이전 실행에서 받은 color_id는 사용하지 않음
            accepted_color_id = None
            color_receiver.color_id = None
            run_start_message_count = color_receiver.message_count

            actual_name = "BLUE" if random_color_id == 1 else "GREEN"

            print()
            print(f"   RANDOM CUBE  {actual_name}")
            print(f"   WAIT ROS     {COLOR_TOPIC}")
            print()

        if is_playing:
            # Play 이후 새로 들어온 유효 color_id를 한 번만 채택
            if (
                accepted_color_id is None
                and step >= COLOR_ACCEPT_DELAY_STEPS
                and color_receiver.message_count > run_start_message_count
                and color_receiver.color_id in PLACE_TARGETS
            ):
                accepted_color_id = color_receiver.color_id
                place_xy = PLACE_TARGETS[accepted_color_id]
                fsm.set_place(place_xy)

                detected_name = (
                    "BLUE"
                    if accepted_color_id == 1
                    else "GREEN"
                )

                print()
                print(
                    f"   DETECTED     "
                    f"{detected_name} ({accepted_color_id})"
                )
                print(f"   PLACE XY     {vec(place_xy)}")
                print()

            # LIFT 완료 후 MOVE 직전에 아직 결과가 없으면 대기
            if (
                fsm.state == fsm.MOVE_STATE
                and accepted_color_id is None
            ):
                robot.apply_action(
                    robot.gripper.forward(action="close")
                )

                if step % LOG_INTERVAL == 0:
                    print(f"   WAIT_COLOR   {COLOR_TOPIC}")

                step += 1
                was_playing = is_playing
                continue

            # 기존 Pick & Place
            target_tcp = fsm.current_target()
            flange_target = tcp_to_flange(
                target_tcp,
                target_quat,
            )

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )

            if solved:
                robot.apply_action(action)

            robot.apply_action(
                robot.gripper.forward(action=fsm.gripper)
            )

            fsm.advance()

            if step % LOG_INTERVAL == 0:
                print_status(
                    robot,
                    solved,
                    fsm,
                    target_tcp,
                )

            step += 1

        was_playing = is_playing

    color_receiver.node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    simulation_app.close()


if __name__ == "__main__":
    main()
