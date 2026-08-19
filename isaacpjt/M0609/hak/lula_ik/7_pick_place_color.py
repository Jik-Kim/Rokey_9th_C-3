"""
Pick & Place + ROS 색상 감지 — 색을 보고 놓을 곳을 정한다

    isaac_python 7_pick_place_color.py

6단계까지는 놓을 곳이 코드에 박혀 있었다.
여기서는 놓을 곳을 다른 PC 가 알려 준다.

    Isaac Sim (PC A)  --/rgb-->       색상 감지 노드 (PC B)
    Isaac Sim (PC A)  <--/color_id--  색상 감지 노드 (PC B)

  1. 파랑/초록 큐브 둘 중 하나를 pick 영역 랜덤 위치로 옮긴다
  2. 큐브 위로 올라가 손목 카메라가 큐브를 보게 한다
  3. PC B 가 보내 준 /color_id 를 기다린다   1=파랑  2=초록
  4. 그 색 마커 위에 놓는다
  5. 다시 1번으로

/rgb 발행은 USD 안의 ROS2 Action Graph 가 한다. 이 코드는 구독만 한다.

왜 rclpy 를 안 쓰는가
    Isaac Sim 5.1 의 파이썬은 3.11 이고 Jazzy 의 rclpy 는 3.12 용으로 빌드돼 있다.
    그래서 이 스크립트 안에서 `import rclpy` 를 하면
        ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
    로 죽는다. Isaac Sim 안에는 rclpy 가 아예 없고 C/C++ 메시지 라이브러리만 들어 있다.
    대신 ROS2 브릿지가 C++ 로 제공하는 OmniGraph 노드 ROS2Subscriber 로 구독한다.

실행 전에 브릿지 라이브러리 경로를 잡아야 한다 (.bashrc 의 isaac_ros)
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
        $HOME/isaac-sim-5.1.0/exts/isaacsim.ros2.bridge/jazzy/lib
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ROS2 브릿지는 앱이 뜬 다음에 켠다.
# USD 안의 Camera Publish 그래프도 이 확장이 있어야 /rgb 를 쏜다
from isaacsim.core.utils.extensions import enable_extension

ROS2_BRIDGE_OK = enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

if not ROS2_BRIDGE_OK:
    print("\n  isaacsim.ros2.bridge 를 못 켰다."
          "\n  터미널에서 isaac_ros 를 먼저 실행했는지 확인한다\n")

from pathlib import Path
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)

# ROS2 구독은 OmniGraph 노드로 한다.  rclpy 는 쓰지 않는다 (파일 맨 위 설명 참고)
import omni.graph.core as og


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH         = str(M0609_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


# ══════════════════════════════════════════════════════════════
#  로봇 설정 — 6단계와 같다
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

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

GRIPPER_JOINTS    = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.8

FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  색상 코드 — PC B(m0609_color_detector.py)와 반드시 같아야 한다
# ══════════════════════════════════════════════════════════════
COLOR_NONE  = 0
COLOR_BLUE  = 1
COLOR_GREEN = 2

COLOR_NAMES = {COLOR_NONE: "NONE", COLOR_BLUE: "BLUE", COLOR_GREEN: "GREEN"}
COLOR_KR    = {COLOR_NONE: "미감지", COLOR_BLUE: "파랑", COLOR_GREEN: "초록"}

COLOR_TOPIC = "/color_id"

# 큐브·마커에 칠할 RGB (0~1).  채도를 높게 잡아야 HSV 판별이 쉽다
RGB = {
    COLOR_BLUE:  np.array([0.05, 0.15, 0.95]),
    COLOR_GREEN: np.array([0.05, 0.85, 0.15]),
}


# ══════════════════════════════════════════════════════════════
#  큐브와 마커
# ══════════════════════════════════════════════════════════════
CUBE_SIZE = 0.05                     # 한 변 (m).  PICK_Z 와 짝이 맞아야 한다
CUBE_MASS = 0.05

# pick 영역 — 매 사이클 이 사각형 안에서 랜덤으로 뽑는다
#
# 마커와 얼마나 떨어뜨려야 하는지는 손목 카메라 화각이 정한다. 실측하면
#   RealSense D455 화각 90.53 deg  (640x640 이라 가로세로 같다)  tan(fov/2) = 1.009
#   WAIT_COLOR 자세에서 카메라는 TCP + [-0.011, -0.045, +0.144] 에 있고
#   바닥까지 0.394 m 라 한 변 0.80 m 를 본다
# 마커가 이 사각형에 걸치면 감지 노드가 큐브 대신 마커를 센다.
# 마커(0.08 m)가 큐브 윗면(0.05 m)보다 커서 픽셀 수로는 마커가 이긴다.
#   -> 카메라 중심에서 0.398 + 마커 반변 0.04 = 0.438 m 밖에 마커를 둔다
PICK_AREA_X = (0.28, 0.42)
PICK_AREA_Y = (0.10, 0.24)

# 쉬는 큐브를 치워 둘 자리.  카메라가 절대 보지 않는 뒤쪽이다
PARK_XY = np.array([-0.45, 0.45])

# 색깔별 place 위치 — 여기에 같은 색 마커를 깔아 둔다
#   pick 영역 y 하한 0.10 -> 카메라 y 0.055, 마커 윗변 -0.45+0.04 = -0.41
#   둘의 거리 0.465 > 0.398 이라 마커는 화면 밖이다
PLACE_XY = {
    COLOR_BLUE:  np.array([0.45, -0.45]),
    COLOR_GREEN: np.array([0.28, -0.45]),
}

MARKER_SIZE = 0.08                   # 마커 한 변
MARKER_THICK = 0.002                 # 바닥에 붙은 판


# ══════════════════════════════════════════════════════════════
#  높이 / 속도 — 6단계와 같다
# ══════════════════════════════════════════════════════════════
PICK_Z          = 0.05
PLACE_Z         = 0.055
APPROACH_HEIGHT = 0.25
LIFT_HEIGHT     = 0.23

GRIPPER_WAIT = 120

TCP_SPEED  = 0.004
MIN_STEPS  = 60
MAX_STEPS  = 600

APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0


# ══════════════════════════════════════════════════════════════
#  색 대기 규칙
# ══════════════════════════════════════════════════════════════
# 큐브 위에 올라가 카메라가 안정된 뒤부터 센다
COLOR_SETTLE_STEPS = 45

# 같은 값이 이만큼 연속으로 와야 믿는다.  PC B 도 자체 안정화를 하지만
# 이동 중 프레임이 섞여 들어오는 것을 한 번 더 거른다
COLOR_STABLE_COUNT = 8

# 직전 사이클에서 받아 둔 값과 같은 값이면 이만큼은 기다린 뒤에 확정한다.
#
# outputs:data 는 마지막 수신값을 계속 들고 있다. 새 큐브를 스폰한 직후에도
# 직전 사이클의 색이 그대로 남아 있어서, 그냥 연속 판정만 하면
# PC B 가 새 화면을 보기도 전에 옛날 색으로 확정해 버린다.
# 값이 바뀌었다면 새 프레임을 처리했다는 증거이므로 바로 믿어도 된다.
COLOR_STALE_HOLD_STEPS = 180

# 이 스텝 동안 못 받으면 경고를 찍고 스폰한 색으로 진행한다.
# 데모가 멈춰 서지 않게 하려는 것이고, 실패는 로그와 통계에 남긴다
COLOR_TIMEOUT_STEPS = 900

CYCLES = 0          # 0 이면 창을 닫을 때까지 무한 반복


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — 6단계와 같다
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    """쿼터니언 곱. 순서는 (w, x, y, z)"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis(axis, deg):
    """회전축과 각도(도)로 쿼터니언을 만든다"""
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    """roll, pitch 로 접근 방향을 정하고 yaw 로 손가락 방향만 돌린다"""
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """쿼터니언을 회전행렬로. 각 열이 로컬 축의 월드 방향이다"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ══════════════════════════════════════════════════════════════
#  TCP 변환 — 6단계와 같다
# ══════════════════════════════════════════════════════════════
def tcp_to_flange(tcp_pos, quat):
    """손가락 끝 목표를 플랜지 목표로 바꾼다"""
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(robot):
    """현재 플랜지 pose 로부터 손가락 끝의 월드 위치를 구한다"""
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  궤적 보간 — 6단계와 같다
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    """구간 길이를 속도로 나눠 스텝 수를 정한다"""
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    """시작점에서 목표점까지 선형 보간"""
    return start + alpha * (goal - start)


# ══════════════════════════════════════════════════════════════
#  ROS — /color_id 구독 (OmniGraph)
# ══════════════════════════════════════════════════════════════
class ColorSubscriber:
    """
    PC B 가 보내는 색상 코드를 받아 들고만 있는다.

    rclpy 대신 브릿지의 ROS2Subscriber 노드를 쓴다.
    이 노드는 아무 ROS2 메시지나 구독할 수 있고,
    메시지 종류를 정해 주면 그에 맞는 출력 속성을 스스로 만든다.
    std_msgs/msg/Int32 는 outputs:data 하나가 생긴다.

        OnPlaybackTick ──tick──> ROS2Subscriber(/color_id) ──> outputs:data

    Play 중에만 tick 이 돈다. FSM 도 Play 중에만 도니 짝이 맞는다.

    주의: world.step(render=False) 로 돌리면 tick 이 오지 않아 값이 0 에서 멈춘다.
    반드시 render=True 로 스텝해야 한다 (headless 여도 상관없다).
    """

    GRAPH_PATH = "/ColorGraph"

    def __init__(self, topic=COLOR_TOPIC):
        self._node = None
        self._attr = None
        self.reads = 0              # outputs:data 를 읽어 낸 횟수
        self.seen = set()           # 지금까지 본 값들 — 진단용
        self.last = COLOR_NONE
        self._build(topic)

    def _build(self, topic):
        """구독 그래프를 만든다. USD 안의 카메라 그래프와는 별개다"""
        keys = og.Controller.Keys

        # 토픽 이름은 앞의 / 를 뗀다. nodeNamespace 가 비어 있으면 전역 토픽이 된다
        name = topic.lstrip("/")

        (_, nodes, _, _) = og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("Subscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "Subscriber.inputs:execIn"),
                ],
                keys.SET_VALUES: [
                    ("Subscriber.inputs:topicName", name),
                    # 아래 셋으로 메시지 종류가 정해지고 출력 속성이 생긴다.
                    # messageName 을 마지막에 넣어야 한다
                    ("Subscriber.inputs:messagePackage", "std_msgs"),
                    ("Subscriber.inputs:messageSubfolder", "msg"),
                    ("Subscriber.inputs:messageName", "Int32"),
                ],
            },
        )
        self._node = nodes[-1]

        # 출력 속성이 만들어질 시간을 준다
        for _ in range(5):
            simulation_app.update()

        self._resolve()
        print(f"   subscribe    /{name}  std_msgs/msg/Int32"
              f"   ({'ok' if self._attr is not None else 'outputs:data 없음'})")

    def _resolve(self):
        """동적으로 생긴 outputs:data 를 잡는다"""
        try:
            self._attr = og.Controller.attribute("outputs:data", self._node)
        except Exception:                               # noqa: BLE001
            self._attr = None

    def poll(self):
        """물리 스텝마다 한 번 불러 최신 값을 읽는다.

        속성은 마지막으로 받은 값을 계속 들고 있다.
        아직 아무것도 안 왔으면 기본값 0 이라 '미수신' 과 '0(미감지)' 은 구분되지 않는다.
        """
        if self._attr is None:
            self._resolve()
        if self._attr is None:
            return self.last

        try:
            value = int(self._attr.get())
        except Exception:                               # noqa: BLE001
            return self.last

        self.reads += 1
        self.seen.add(value)
        self.last = value
        return value

    @property
    def got_signal(self):
        """0 말고 다른 값을 한 번이라도 본 적이 있는가"""
        return bool(self.seen - {COLOR_NONE})


# ══════════════════════════════════════════════════════════════
#  상태 기계 — 6단계에 WAIT_COLOR 와 RETREAT 를 넣었다
# ══════════════════════════════════════════════════════════════
S_APPROACH = 0
S_WAIT     = 1
S_DESCEND  = 2
S_GRASP    = 3
S_LIFT     = 4
S_MOVE     = 5
S_LOWER    = 6
S_RELEASE  = 7
S_RETREAT  = 8
S_DONE     = 9


class ColorPickPlaceFSM:
    """
      0 APPROACH    큐브 위로 접근
      1 WAIT_COLOR  제자리에서 /color_id 를 기다린다   ← 6단계에 없던 단계
      2 DESCEND     큐브까지 하강
      3 GRASP       그리퍼 닫기 (제자리)
      4 LIFT        들어올리기
      5 MOVE        받은 색의 마커 위로 이동          ← 목표가 런타임에 정해진다
      6 LOWER       놓을 높이까지 하강
      7 RELEASE     그리퍼 열기 (제자리)
      8 RETREAT     큐브에서 손을 빼며 위로
      9 DONE

    6단계는 waypoints 를 리스트로 미리 만들어 두었다.
    여기서는 pick 이 랜덤이고 place 는 색을 받아야 정해지므로
    단계에 들어가는 순간 계산한다.
    """

    NAMES = ["APPROACH", "WAIT_COLOR", "DESCEND", "GRASP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "RETREAT", "DONE"]
    GRIPPER_STATES = {S_GRASP: "close", S_RELEASE: "open"}
    DONE_STATE = S_DONE

    def __init__(self, robot, color_client):
        self._robot = robot
        self._color = color_client
        self.reset(np.array([0.0, 0.0]))

    # ── 사이클 시작 ─────────────────────────────────────
    def reset(self, pick_xy):
        self.pick_xy   = np.asarray(pick_xy, dtype=float)
        self.place_xy  = None          # 색을 받아야 정해진다
        self.detected  = COLOR_NONE
        self.timed_out = False
        self.stale     = self._color.last   # 직전 사이클에서 남은 값

        self.state   = S_APPROACH
        self.step    = 0
        self.start   = None
        self.goal    = self._goal_for(S_APPROACH)
        self.n_steps = MIN_STEPS
        self.gripper = "open"

        # WAIT_COLOR 용
        self._wait_step = 0
        self._last_seen = COLOR_NONE
        self._streak    = 0

    # ── 목표 ────────────────────────────────────────────
    def _goal_for(self, state):
        """단계별 TCP 목표.  place 좌표는 색이 정해진 뒤에만 유효하다"""
        px, py = self.pick_xy
        gx, gy = self.place_xy if self.place_xy is not None else self.pick_xy

        return np.array({
            S_APPROACH: [px, py, APPROACH_HEIGHT],
            S_WAIT:     [px, py, APPROACH_HEIGHT],
            S_DESCEND:  [px, py, PICK_Z],
            S_GRASP:    [px, py, PICK_Z],
            S_LIFT:     [px, py, LIFT_HEIGHT],
            S_MOVE:     [gx, gy, LIFT_HEIGHT],
            S_LOWER:    [gx, gy, PLACE_Z],
            S_RELEASE:  [gx, gy, PLACE_Z],
            S_RETREAT:  [gx, gy, LIFT_HEIGHT],
        }[state], dtype=float)

    def current_target(self):
        """이번 스텝의 TCP 목표"""
        # WAIT_COLOR 는 n_steps 가 0 이다. 보간할 것이 없으니 제자리를 지킨다
        if self.start is None or self.n_steps <= 0:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    # ── 진행 ────────────────────────────────────────────
    def advance(self):
        """한 스텝 진행한다"""
        if self.state >= S_DONE:
            return

        if self.start is None:
            self._enter_state()

        if self.state == S_WAIT:
            self._wait_for_color()
            return

        self.step += 1
        if self.step >= self.n_steps:
            self._next()

    def _enter_state(self):
        """단계에 처음 들어온 순간 시작점과 스텝 수를 정한다"""
        self.start = get_tcp_pose(self._robot)
        self.goal  = self._goal_for(self.state)
        self.gripper = self.GRIPPER_STATES.get(self.state, self.gripper)

        if self.state == S_WAIT:
            self.n_steps, dist = 0, 0.0
        elif self.state in self.GRIPPER_STATES:
            self.n_steps, dist = GRIPPER_WAIT, 0.0
        else:
            self.n_steps, dist = steps_for(self.start, self.goal)

        print(f"   [{self.state}] {self.NAMES[self.state]:10s}"
              f" goal {vec(self.goal)}"
              f"  {dist:.4f} m  {self.n_steps} steps  gripper {self.gripper}")

    def _wait_for_color(self):
        """
        제자리에 떠서 /color_id 를 본다.

        카메라가 흔들리는 동안의 프레임은 버리고,
        같은 값이 COLOR_STABLE_COUNT 번 연속 와야 확정한다.
        직전 사이클과 같은 값이면 COLOR_STALE_HOLD_STEPS 까지 더 기다린다.
        """
        self._wait_step += 1
        seen = self._color.poll()

        if self._wait_step < COLOR_SETTLE_STEPS:
            return

        if seen == self._last_seen and seen != COLOR_NONE:
            self._streak += 1
        else:
            self._last_seen = seen
            self._streak = 1 if seen != COLOR_NONE else 0

        if self._streak >= COLOR_STABLE_COUNT:
            # 값이 바뀌었으면 PC B 가 새 화면을 처리했다는 뜻이라 바로 믿는다.
            # 직전 값 그대로면 아직 안 바뀐 것인지 알 수 없으니 더 기다린다
            fresh = (seen != self.stale)
            if fresh or self._wait_step >= COLOR_STALE_HOLD_STEPS:
                self._confirm(seen, timed_out=False)
            return

        # 타임아웃 처리는 한 번만. 메인 루프가 다음 스텝에 색을 밀어 넣는다
        if self._wait_step >= COLOR_TIMEOUT_STEPS and not self.timed_out:
            self._confirm(COLOR_NONE, timed_out=True)

    def _confirm(self, color, timed_out):
        """색을 확정하고 place 좌표를 연다"""
        self.timed_out = timed_out
        self.detected  = color

        if timed_out:
            hint = ("PC B 가 계속 0(미감지)을 보낸다"
                    if self._color.got_signal else
                    "/color_id 에서 0 말고 다른 값을 한 번도 못 봤다")
            print(f"       색 확정 실패 {self._wait_step} steps — {hint}")
            return          # place_xy 는 메인 루프가 대신 정해 준다

        self.place_xy = PLACE_XY[color]
        why = "값 바뀜" if color != self.stale else f"{self._wait_step} steps 대기"
        print(f"       color_id = {color} {COLOR_NAMES[color]}"
              f"  ({COLOR_KR[color]}) -> place {vec(self.place_xy)}   [{why}]")
        self._next()

    def force_color(self, color):
        """타임아웃 때 메인 루프가 색을 밀어 넣는다"""
        self.detected = color
        self.place_xy = PLACE_XY[color]
        print(f"       [FALLBACK] 스폰 색 {COLOR_NAMES[color]} 으로 진행")
        self._next()

    def _next(self):
        self.state += 1
        self.step  = 0
        self.start = None
        if self.state >= S_DONE:
            print(f"   [{S_DONE}] DONE")

    @property
    def waiting_for_fallback(self):
        """타임아웃이 났는데 아직 색이 안 정해진 상태인가"""
        return self.state == S_WAIT and self.timed_out and self.place_xy is None


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
def find_prim_path(root_path, name):
    """USD 계층에서 이름으로 prim 경로를 찾는다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


class M0609ColorTask(BaseTask):
    """
    6단계 Task 에 큐브 두 개와 마커 두 개를 더한다.

    USD 에 들어 있는 blue_block 은 숨긴다.
    파란 큐브가 화면에 둘이면 카메라가 어느 쪽을 본 것인지 알 수 없다.
    """

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None
        self._cubes = {}

    # ── 프레임워크 규약 ──────────────────────────────────
    def set_up_scene(self, scene):
        """world.reset() 안에서 자동으로 불린다"""
        super().set_up_scene(scene)
        self._load_usd()
        self._hide_usd_block()
        self._setup_arm_drives()
        self._register_robot(scene)
        self._add_markers(scene)
        self._add_cubes(scene)
        print("   scene        ready")

    # ── 우리가 나눈 단계 ─────────────────────────────────
    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()

        print("   USD          loaded")

    def _hide_usd_block(self):
        """USD 에 원래 있던 파란 큐브를 감춘다"""
        path = find_prim_path("/World", "blue_block")
        if path is None:
            print("   usd block    none")
            return

        stage = omni.usd.get_context().get_stage()
        UsdGeom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()
        print(f"   usd block    hidden  {path}")

    def _setup_arm_drives(self):
        """IK 결과를 로봇이 따라가도록 팔 관절의 Drive 를 강화한다"""
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
        """로봇과 그리퍼를 등록한다"""
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

    def _add_markers(self, scene):
        """
        놓을 자리를 눈으로 보이게 깐다.

        VisualCuboid 라 충돌체가 없다.  큐브를 놓을 때 걸리지 않는다.
        """
        for color, xy in PLACE_XY.items():
            name = COLOR_NAMES[color].lower()
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/marker_{name}",
                    name=f"marker_{name}",
                    position=np.array([xy[0], xy[1], MARKER_THICK / 2.0]),
                    scale=np.array([MARKER_SIZE, MARKER_SIZE, MARKER_THICK]),
                    color=RGB[color],
                )
            )
            print(f"   marker       {name:5s} {vec(xy)}")

    def _add_cubes(self, scene):
        """파랑·초록 큐브를 만든다. 시작 위치는 둘 다 대기 자리다"""
        for color in (COLOR_BLUE, COLOR_GREEN):
            name = COLOR_NAMES[color].lower()
            self._cubes[color] = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/cube_{name}",
                    name=f"cube_{name}",
                    position=np.array([PARK_XY[0], PARK_XY[1], CUBE_SIZE / 2.0]),
                    scale=np.array([CUBE_SIZE] * 3),
                    color=RGB[color],
                    mass=CUBE_MASS,
                )
            )
            print(f"   cube         {name:5s} size {CUBE_SIZE}")

    @property
    def robot(self):
        return self._robot

    @property
    def cubes(self):
        return self._cubes


# ══════════════════════════════════════════════════════════════
#  큐브 배치
# ══════════════════════════════════════════════════════════════
def place_cube(cube, xy, z):
    """큐브를 순간이동시키고 속도를 지운다.

    속도를 안 지우면 이전에 굴러가던 힘이 남아 새 자리에서 튄다.
    """
    cube.set_world_pose(
        position=np.array([xy[0], xy[1], z]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    cube.set_linear_velocity(np.zeros(3))
    cube.set_angular_velocity(np.zeros(3))


def spawn_cycle(cubes, rng):
    """
    두 큐브 중 하나를 골라 pick 영역 랜덤 위치로 옮기고
    나머지는 뒤쪽 대기 자리로 치운다.

    반환값은 (고른 색, 놓은 xy) — 정답이다. 감지 결과와 비교하는 데 쓴다
    """
    color = COLOR_BLUE if rng.random() < 0.5 else COLOR_GREEN
    xy = np.array([
        rng.uniform(*PICK_AREA_X),
        rng.uniform(*PICK_AREA_Y),
    ])

    for c, cube in cubes.items():
        target = xy if c == color else PARK_XY
        place_cube(cube, target, CUBE_SIZE / 2.0)

    return color, xy


# ══════════════════════════════════════════════════════════════
#  로봇 초기화 — 6단계와 같다
# ══════════════════════════════════════════════════════════════
def init_gripper(robot, world):
    """그리퍼는 Articulation 초기화 이후에 따로 초기화한다"""
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )


def set_ready_pose(robot):
    """시작 자세로 보낸다"""
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


def create_ik_solver(robot):
    """Lula 계산기를 만들고 로봇과 연결한다"""
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
    """벡터를 고정폭으로 찍는다"""
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def print_plan(target_quat):
    """무엇을 할지 먼저 확인한다"""
    R = quat_to_matrix(target_quat)

    section("PLAN")
    print(f"   pick area    x {PICK_AREA_X}   y {PICK_AREA_Y}")
    print(f"   park xy      {vec(PARK_XY)}")
    for color, xy in PLACE_XY.items():
        print(f"   place {COLOR_NAMES[color]:5s}  {vec(xy)}   color_id = {color}")
    print()
    print(f"   subscribe    {COLOR_TOPIC}  std_msgs/Int32")
    print(f"   settle       {COLOR_SETTLE_STEPS} steps")
    print(f"   stable       {COLOR_STABLE_COUNT} 회 연속")
    print(f"   stale hold   {COLOR_STALE_HOLD_STEPS} steps  (직전 값과 같을 때)")
    print(f"   timeout      {COLOR_TIMEOUT_STEPS} steps")
    print()
    print(f"   approach z   {APPROACH_HEIGHT}")
    print(f"   pick z       {PICK_Z}    place z {PLACE_Z}    lift z {LIFT_HEIGHT}")
    print(f"   tcp speed    {TCP_SPEED} m/step")
    print()
    print(f"   tool   +Z    {vec(R @ np.array([0, 0, 1]))}   approach direction")
    print(f"   finger +X    {vec(R @ np.array([1, 0, 0]))}   finger direction")


def print_status(robot, solved, fsm, target_tcp):
    """현재 단계와 손가락 끝 위치를 함께 찍는다"""
    name = fsm.NAMES[min(fsm.state, S_DONE)]

    if not solved:
        print(f"   {name:10s} IK FAILED  target {vec(target_tcp)}")
        return

    tcp = get_tcp_pose(robot)
    finger = robot.get_joint_positions()[robot.get_dof_index("finger_joint")]
    print(f"   {name:10s} tcp {vec(tcp)}   finger {finger:+.4f}")


class Score:
    """감지가 맞았는지 세어 둔다.  스폰한 색을 알고 있으니 채점이 된다"""

    def __init__(self):
        self.total = 0
        self.hit = 0
        self.miss = 0
        self.timeout = 0

    def add(self, spawned, detected, timed_out):
        self.total += 1
        if timed_out:
            self.timeout += 1
        elif detected == spawned:
            self.hit += 1
        else:
            self.miss += 1

    def line(self):
        return (f"cycle {self.total}   정답 {self.hit}"
                f"  오판 {self.miss}  미수신 {self.timeout}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = M0609ColorTask(name="m0609_color_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    init_gripper(robot, world)
    set_ready_pose(robot)
    for _ in range(30):
        world.step(render=True)

    section("ROS")
    color_client = ColorSubscriber()

    section("SOLVER")
    ik_solver = create_ik_solver(robot)
    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )
    print_plan(target_quat)

    section("RUN")
    print("   press Play in the viewport")
    print("   PC B 에서 `ros2 run m0609 m0609_color_detector` 를 함께 띄운다\n")

    rng = np.random.default_rng()
    fsm = ColorPickPlaceFSM(robot, color_client)
    score = Score()

    spawned = COLOR_NONE
    was_playing = False
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        # 물리 스텝마다 한 번씩 ROS 큐를 비운다
        color_client.poll()

        is_playing = world.is_playing()

        # Play 를 누른 순간 처음부터 시작한다
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            init_gripper(robot, world)
            set_ready_pose(robot)
            spawned, pick_xy = spawn_cycle(task.cubes, rng)
            fsm.reset(pick_xy)
            step = 0
            print(f"\n── CYCLE {score.total + 1} "
                  f"  spawn {COLOR_NAMES[spawned]} at {vec(pick_xy)} ──")

        if is_playing:
            # 팔 — 이번 스텝의 목표를 보간으로 구해 IK 로 푼다
            target_tcp = fsm.current_target()
            flange_target = tcp_to_flange(target_tcp, target_quat)

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )
            if solved:
                robot.apply_action(action)

            # 그리퍼 — 현재 단계가 정한 상태를 유지한다
            robot.apply_action(robot.gripper.forward(action=fsm.gripper))

            fsm.advance()

            # 색을 못 받았으면 스폰한 색으로 밀고 나간다
            if fsm.waiting_for_fallback:
                fsm.force_color(spawned)

            # 한 사이클이 끝났다 — 채점하고 다시 스폰한다
            if fsm.state >= S_DONE:
                score.add(spawned, fsm.detected, fsm.timed_out)
                mark = "OK " if fsm.detected == spawned and not fsm.timed_out else "NG "
                print(f"   {mark} spawn {COLOR_NAMES[spawned]:5s}"
                      f"  detect {COLOR_NAMES[fsm.detected]:5s}"
                      f"   {score.line()}")

                if CYCLES and score.total >= CYCLES:
                    print("\n   요청한 사이클을 다 돌았다\n")
                    break

                spawned, pick_xy = spawn_cycle(task.cubes, rng)
                fsm.reset(pick_xy)
                print(f"\n── CYCLE {score.total + 1} "
                      f"  spawn {COLOR_NAMES[spawned]} at {vec(pick_xy)} ──")

            if step % LOG_INTERVAL == 0:
                print_status(robot, solved, fsm, target_tcp)
            step += 1

        was_playing = is_playing

    section("RESULT")
    print(f"   {score.line()}")
    print(f"   /color_id 읽기 {color_client.reads} 회"
          f"   본 값 {sorted(color_client.seen)}\n")

    simulation_app.close()


if __name__ == "__main__":
    main()
