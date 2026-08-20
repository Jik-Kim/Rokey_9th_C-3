"""
PC A 최종본 — Isaac Sim Standalone Pick & Place + ROS 2

- Wrist Camera -> /rgb (sensor_msgs/msg/Image)
- /color_id (std_msgs/msg/Int32) 구독
- 1 -> BLUE place / 2 -> GREEN place
- Play 시작마다 Pick cube를 BLUE/GREEN 중 하나로 랜덤 설정
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
# import omni.graph.core as og
# import omni.replicator.core as rep

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
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
PICK_XY = np.array([0.25, 0.10])

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
MARKER_SIZE = 0.05        # 12 cm x 12 cm
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


def find_pick_cube_prim():
    stage = omni.usd.get_context().get_stage()
    candidates = []

    for prim in stage.Traverse():
        path = str(prim.GetPath())

        if path.startswith(ROBOT_PRIM_PATH + "/"):
            continue

        # Place 바닥 마커는 Pick cube 탐색 대상에서 제외
        if path.startswith("/World/PlaceMarkers/"):
            continue

        name = prim.GetName().lower()

        if not (prim.IsA(UsdGeom.Cube) or "cube" in name):
            continue

        try:
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            p = matrix.ExtractTranslation()
            xy = np.array([float(p[0]), float(p[1])])
            dist = float(np.linalg.norm(xy - PICK_XY))
        except Exception:
            dist = 9999.0

        penalty = 0.0 if name == "cube" else 0.05
        candidates.append((dist + penalty, prim))

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


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


# def setup_rgb_publisher(camera_prim_path):
#     render_product = rep.create.render_product(
#         camera_prim_path,
#         CAMERA_RESOLUTION,
#     )

#     K = og.Controller.Keys

#     og.Controller.edit(
#         {
#             "graph_path": "/ROS2CameraGraph",
#             "evaluator_name": "execution",
#         },
#         {
#             K.CREATE_NODES: [
#                 ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
#                 ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
#                 ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
#             ],
#             K.CONNECT: [
#                 ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
#                 ("ROS2Context.outputs:context", "CameraHelper.inputs:context"),
#             ],
#             K.SET_VALUES: [
#                 ("ROS2Context.inputs:domain_id", ROS_DOMAIN_ID),
#                 ("CameraHelper.inputs:renderProductPath", render_product.path),
#                 ("CameraHelper.inputs:topicName", RGB_TOPIC),
#                 ("CameraHelper.inputs:frameId", CAMERA_FRAME_ID),
#                 ("CameraHelper.inputs:type", "rgb"),
#             ],
#         },
#     )

    # for _ in range(10):
    #     simulation_app.update()

    # print(f"   camera       {camera_prim_path}")
    # print(f"   publish      {RGB_TOPIC}  sensor_msgs/msg/Image")
    # print(f"   resolution   {CAMERA_RESOLUTION[0]} x {CAMERA_RESOLUTION[1]}")

    # return render_product


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

    cube_prim = find_pick_cube_prim()
    if cube_prim is None:
        raise RuntimeError("Pick cube를 찾지 못했습니다.")

    section("ROS2")

    color_receiver = ColorReceiver()
    # render_product = setup_rgb_publisher(camera_path)

    section("SOLVER")

    ik_solver = create_ik_solver(robot)

    target_quat = make_target_quat(
        APPROACH_ROLL_DEG,
        APPROACH_PITCH_DEG,
        GRIPPER_YAW_DEG,
    )

    section("PLAN")
    print(f"   pick xy      {vec(PICK_XY)}")
    print(f"   blue  (1)    {vec(BLUE_PLACE_XY)}")
    print(f"   green (2)    {vec(GREEN_PLACE_XY)}")
    print(f"   marker size  {MARKER_SIZE:.3f} m square")
    print(f"   rgb topic    {RGB_TOPIC}")
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

            # Pick cube를 파랑/초록 중 랜덤으로 설정
            random_color_id = random.choice([1, 2])
            set_cube_color(cube_prim, random_color_id)

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

    # _ = render_product
    simulation_app.close()


if __name__ == "__main__":
    main()
