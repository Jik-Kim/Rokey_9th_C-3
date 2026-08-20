"""
1차 목표 — 한 라인 End-to-End

  박스 스폰 -> 3D 인식 -> 하이트맵 패킹으로 배치 결정 -> 흡착 픽
  -> 팔레트에 플레이스 -> 하이트맵 갱신 -> 다음 박스

실행:
    ~/isaacsim/python.sh run_line.py

Viewport 에서 Play 를 누르면 시작합니다.
설정은 전부 config.py 에 있습니다.
"""

import os

from isaacsim import SimulationApp

import config as C

simulation_app = SimulationApp({"headless": C.HEADLESS})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.robot.manipulators.manipulators import SingleManipulator  # noqa: E402
from isaacsim.robot_motion.motion_generation import (  # noqa: E402
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from pxr import Usd, UsdPhysics  # noqa: E402

import perception  # noqa: E402
import scene as scene_builder  # noqa: E402
import suction  # noqa: E402
from packing import HeightmapPacker  # noqa: E402


# ─────────────────────────────────────────────────────────────
# 출력
# ─────────────────────────────────────────────────────────────
def section(title):
    print(f"\n{'─' * 70}")
    print(f" {title}")
    print(f"{'─' * 70}")


def vec(v, digits=3):
    return "[" + " ".join(f"{x:+.{digits}f}" for x in np.asarray(v, dtype=float)) + "]"


# ─────────────────────────────────────────────────────────────
# 회전 / TCP
# ─────────────────────────────────────────────────────────────
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
    a /= np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(yaw_deg):
    """흡착판이 아래를 향한 채 yaw 만 도는 자세."""
    q = quat_mul(
        quat_from_axis([1, 0, 0], C.APPROACH_ROLL_DEG),
        quat_from_axis([0, 1, 0], C.APPROACH_PITCH_DEG),
    )
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tcp_to_flange(tcp_pos, quat):
    """TCP(흡착면) 목표 -> link_6 플랜지 목표."""
    return np.array(tcp_pos) - quat_to_matrix(quat) @ C.TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ C.TCP_OFFSET


def lerp(a, b, alpha):
    return a + alpha * (b - a)


def ease(alpha):
    """
    구간 시작·끝에서 속도가 0 이 되는 S 커브 (cosine ease-in/out).

    선형 보간을 쓰면 웨이포인트 코너에서 속도가 한 스텝 만에 방향을 바꾼다.
    0.3 m/s 가 1/60 초에 꺾이면 가속도가 18 m/s^2 이고, 9kg 박스에 162 N 이
    걸려 흡착 한계(157 N)를 넘긴다. 실제 로봇도 사다리꼴/S 커브 속도
    프로파일을 쓰므로 이게 물리적으로도 맞다.
    """
    a = float(np.clip(alpha, 0.0, 1.0))
    return 0.5 * (1.0 - np.cos(np.pi * a))


def steps_for(start, goal):
    dist = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
    return int(np.clip(dist / C.TCP_SPEED, C.MIN_STEPS, C.MAX_STEPS)), dist


# ─────────────────────────────────────────────────────────────
# 픽&플레이스 FSM (박스 한 개 = 한 사이클)
# ─────────────────────────────────────────────────────────────
class CycleFSM:
    APPROACH_PICK = 0
    DESCEND_PICK = 1
    GRIP = 2
    LIFT = 3
    TRANSIT = 4
    DESCEND_PLACE = 5
    RELEASE = 6
    RETREAT = 7
    DONE = 8

    NAMES = ["APPR_PICK", "DESC_PICK", "GRIP", "LIFT",
             "TRANSIT", "DESC_PLACE", "RELEASE", "RETREAT", "DONE"]

    SUCTION = {GRIP: "close", RELEASE: "open"}

    def __init__(self, robot):
        self._robot = robot
        self.waypoints = [np.zeros(3) for _ in range(self.DONE)]
        self.pick_yaw = 0.0
        self.place_yaw = 0.0
        self.state = self.DONE
        self.step = 0
        self.start = None
        self.n_steps = C.MIN_STEPS
        self.suction = "open"

    def plan(self, pick_top_world, pick_yaw_deg, place_top_world, place_yaw_deg,
             stack_top_world_z=0.0, box_h=0.0):
        """
        한 박스에 대한 전체 경로를 깐다.

        이동 높이는 고정값이 아니라 그때그때 필요한 만큼만 든다.
        불필요하게 높이 들면 로봇 리치를 낭비하고 팔레트 바깥 모서리에서
        IK 가 풀리지 않는다.
        """
        px, py, pz = pick_top_world
        gx, gy, gz = place_top_world

        # 박스를 든 채로는 박스 밑면이 적재물 최고점보다 높아야 지나간다
        transit_z = max(
            pz + C.TRANSIT_CLEARANCE,
            gz + C.TRANSIT_CLEARANCE,
            stack_top_world_z + box_h + C.TRANSIT_CLEARANCE,
        )
        transit_z = min(transit_z, C.TRANSIT_Z_MAX)
        self.transit_z = transit_z

        pick_z = pz + C.PICK_APPROACH_GAP
        self.waypoints = [
            np.array([px, py, pz + C.APPROACH_CLEARANCE]),   # APPROACH_PICK
            np.array([px, py, pick_z]),                      # DESCEND_PICK
            np.array([px, py, pick_z]),                      # GRIP
            np.array([px, py, transit_z]),                   # LIFT
            np.array([gx, gy, transit_z]),                   # TRANSIT
            np.array([gx, gy, gz + C.PLACE_RELEASE_GAP]),    # DESCEND_PLACE
            np.array([gx, gy, gz + C.PLACE_RELEASE_GAP]),    # RELEASE
            np.array([gx, gy, transit_z]),                   # RETREAT
        ]
        self.pick_yaw = float(pick_yaw_deg)
        self.place_yaw = float(place_yaw_deg)

        self.state = self.APPROACH_PICK
        self.step = 0
        self.start = None
        self.suction = "open"

    @property
    def done(self):
        return self.state >= self.DONE

    def current_yaw(self):
        """TRANSIT 중에 픽 자세 -> 플레이스 자세로 손목을 돌린다."""
        if self.state < self.TRANSIT:
            return self.pick_yaw
        if self.state > self.TRANSIT:
            return self.place_yaw
        alpha = min(1.0, self.step / float(max(1, self.n_steps)))
        return lerp(self.pick_yaw, self.place_yaw, ease(alpha))

    def current_target(self):
        # 상태가 막 바뀐 첫 프레임에는 보간 시작점이 아직 없다. 이때 최종
        # 웨이포인트를 그대로 지령하면 최대 1.3m 떨어진 곳으로 순간이동
        # 명령이 나가고, 그 관성으로 흡착이 끊긴다. 현재 위치를 유지한다.
        if self.start is None:
            return get_tcp_pose(self._robot)
        alpha = ease(min(1.0, self.step / float(self.n_steps)))

        if self.state == self.TRANSIT:
            return self._arc_target(alpha)
        return lerp(self.start, self.goal, alpha)

    def _arc_target(self, alpha):
        """
        이송 구간은 직선이 아니라 베이스 중심 호를 그린다.

        픽(방위 -90도)과 플레이스(방위 +90도)를 직선으로 이으면 경로가
        로봇 베이스 바로 위를 지나간다. 그 지점은 어깨 특이점이라 joint_1 이
        폭주하면서 박스를 던져버린다. 실제 팔레타이저도 들어올린 뒤 축을
        돌리고 내려놓는다 — 반경과 방위각을 따로 보간하면 그 동작이 된다.
        """
        base = np.asarray(C.ROBOT_BASE_XY, dtype=float)
        s_xy, g_xy = self.start[:2] - base, self.goal[:2] - base
        r0, r1 = float(np.linalg.norm(s_xy)), float(np.linalg.norm(g_xy))
        a0 = float(np.arctan2(s_xy[1], s_xy[0]))
        a1 = float(np.arctan2(g_xy[1], g_xy[0]))
        da = float(np.arctan2(np.sin(a1 - a0), np.cos(a1 - a0)))  # 짧은 쪽

        r = r0 + alpha * (r1 - r0)
        a = a0 + alpha * da
        z = self.start[2] + alpha * (self.goal[2] - self.start[2])
        return np.array([base[0] + r * np.cos(a), base[1] + r * np.sin(a), z])

    def advance(self, verbose=False):
        if self.done:
            return

        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]
            self.suction = self.SUCTION.get(self.state, self.suction)

            if self.state in self.SUCTION:
                self.n_steps, dist = C.GRIPPER_WAIT, 0.0
            elif self.state == self.TRANSIT:
                base = np.asarray(C.ROBOT_BASE_XY, dtype=float)
                s_xy, g_xy = self.start[:2] - base, self.goal[:2] - base
                r0, r1 = np.linalg.norm(s_xy), np.linalg.norm(g_xy)
                a0, a1 = np.arctan2(*s_xy[::-1]), np.arctan2(*g_xy[::-1])
                da = abs(np.arctan2(np.sin(a1 - a0), np.cos(a1 - a0)))
                dist = float(da * (r0 + r1) / 2.0 + abs(r1 - r0)
                             + abs(self.goal[2] - self.start[2]))
                self.n_steps = int(np.clip(dist / C.TCP_SPEED,
                                           C.MIN_STEPS, C.MAX_STEPS))
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)

            if verbose:
                print(f"      [{self.state}] {self.NAMES[self.state]:10s}"
                      f" -> {vec(self.goal)}  {dist*1000:5.0f} mm"
                      f"  {self.n_steps:3d} steps  suction {self.suction}")

        self.step += 1
        if self.step >= self.n_steps:
            self.state += 1
            self.step = 0
            self.start = None


# ─────────────────────────────────────────────────────────────
# 로봇 셋업
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


def configure_drives(stage):
    """
    IK 목표를 실제로 따라가도록 관절 드라이브를 단단하게 만든다.
    박스를 든 채 자세가 처지면 흡착이 전단력을 못 견디고 떨어진다.
    """
    root = stage.GetPrimAtPath(C.ROBOT_PRIM_PATH)
    if not root.IsValid():
        return 0

    n = 0
    for prim in Usd.PrimRange(root):
        if prim.GetName() not in C.ARM_JOINTS:
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr().Set(float(C.DRIVE_STIFFNESS))
        drive.CreateDampingAttr().Set(float(C.DRIVE_DAMPING))
        drive.CreateMaxForceAttr().Set(float(C.DRIVE_MAX_FORCE))
        n += 1
    return n



def apply_gains(robot):
    """
    관절 게인을 아티큘레이션 컨트롤러로 직접 넣는다.

    USD 의 DriveAPI 속성만 써두면 로봇에 따라 초기화 시점에 덮어써져
    실제로는 게인이 0 인 채로 돌 수 있다 (H2017 에서 IK 목표와 실제 관절이
    20도 벌어진 채 고정되는 현상이 이것 때문이었다).
    initialize() 이후, Play 마다 다시 호출해야 한다.
    """
    ctrl = robot.get_articulation_controller()
    n = robot.num_dof
    ctrl.set_gains(kps=np.full(n, float(C.DRIVE_STIFFNESS)),
                   kds=np.full(n, float(C.DRIVE_DAMPING)))
    ctrl.set_max_efforts(np.full(n, float(C.DRIVE_MAX_FORCE)))
    return ctrl


def set_ready_pose(robot):
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(C.READY_JOINTS_DEG)
    robot.set_joint_positions(q)


def create_ik_solver(robot):
    lula = LulaKinematicsSolver(
        robot_description_path=C.DESCRIPTION_PATH,
        urdf_path=C.URDF_PATH,
    )
    lula.set_robot_base_pose(
        robot_position=C.ROBOT_BASE_POS,
        robot_orientation=C.ROBOT_BASE_QUAT,
    )
    art = ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=C.EE_LINK_NAME,
    )
    return lula, art


def solve_ik(lula, robot, target_pos, target_quat):
    """
    IK 를 풀되 "팔을 앞으로 뻗는" 가지를 강제한다.

    베이스 회전(joint_1)은 목표를 정면으로 보는 각과 그보다 180도 돌아
    어깨 너머로 뒤집어 뻗는 각, 두 해가 항상 존재한다. 후자를 고르면
    몸통이 컨베이어를 뚫고 지나가야 해서 실제로는 도달하지 못한다.
    현재 관절을 warm start 로 쓰되 joint_1 만 목표 방위각으로 갈아끼워
    올바른 가지로 유도한다.
    """
    q = np.array(robot.get_joint_positions()[:6], dtype=float)
    azimuth = float(np.arctan2(target_pos[1] - C.ROBOT_BASE_POS[1],
                               target_pos[0] - C.ROBOT_BASE_POS[0]))

    front = q.copy()
    front[0] = azimuth

    def wrap(a):
        return np.arctan2(np.sin(a), np.cos(a))

    # 현재 자세를 먼저 시드로 쓴다. 매 스텝 방위각으로 새로 시드하면
    # 해가 다른 가지로 튀면서 팔이 급격히 움직이고, 그 관성으로 흡착이 풀린다.
    best_fallback = None
    for seed in (q.copy(), front):
        q_sol, solved = lula.compute_inverse_kinematics(
            frame_name=C.EE_LINK_NAME,
            warm_start=seed,
            target_position=np.asarray(target_pos, dtype=float),
            target_orientation=np.asarray(target_quat, dtype=float),
        )
        if not solved:
            continue
        # 어깨 너머로 뒤집어 뻗는 해는 몸통이 컨베이어를 뚫어야 해서 못 쓴다
        if abs(wrap(q_sol[0] - azimuth)) > np.pi / 2:
            continue
        # 한 스텝에 30도 넘게 뛰는 해는 관성 충격을 준다
        if np.max(np.abs(wrap(q_sol - q))) > np.radians(JUMP_LIMIT_DEG):
            if best_fallback is None:
                best_fallback = q_sol
            continue
        return q_sol, True

    if best_fallback is not None:
        # 멀리 떨어진 해밖에 없으면 그쪽으로 "걸어간다".
        # 그대로 적용하면 팔이 순간이동하듯 튀고, 그 관성으로 흡착이 풀린다.
        lim = np.radians(JUMP_LIMIT_DEG)
        return q + np.clip(wrap(best_fallback - q), -lim, lim), True
    return None, False



def debug_roi_contents(stage, label=""):
    """
    스캔 ROI 안에 실제로 무엇이 들어 있는지 센다.
    PL_DEBUG_ROI=1 일 때만 호출된다.

    카메라가 두 물체를 하나로 인식할 때, 그게 정말 박스 두 개인지
    아니면 단일 박스의 점군 아티팩트인지 가르는 유일한 방법이다.
    """
    from pxr import UsdGeom

    lo_xy = C.PICK_XY - C.CAM_ROI_HALF
    hi_xy = C.PICK_XY + C.CAM_ROI_HALF
    floor_z = C.CONVEYOR_TOP_Z + C.CAM_PLANE_TOL
    ceil_z = C.CONVEYOR_TOP_Z + C.CAM_MAX_BOX_H

    root = stage.GetPrimAtPath(C.BOX_ROOT)
    hits = []
    if root.IsValid():
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Cube):
                continue
            mat = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())).T
            pos = mat[:3, 3]
            scale = np.linalg.norm(mat[:3, :3], axis=0)
            top = pos[2] + scale[2] / 2.0
            inside_xy = np.all(pos[:2] > lo_xy) and np.all(pos[:2] < hi_xy)
            if inside_xy and floor_z < top < ceil_z:
                hits.append((prim.GetName(), pos, scale))

    print(f"        [ROI{label}] 박스 {len(hits)} 개"
          f"   x {lo_xy[0]:.3f}~{hi_xy[0]:.3f}"
          f"  y {lo_xy[1]:.3f}~{hi_xy[1]:.3f}"
          f"  z {floor_z:.3f}~{ceil_z:.3f}")
    for name, pos, scale in hits:
        print(f"          {name}  pos {vec(pos)}"
              f"  size [{scale[0]*1000:.0f} {scale[1]*1000:.0f} {scale[2]*1000:.0f}] mm")
    return hits



REJECT_POS = np.array([0.0, -2.0, 0.05])   # 씬 밖 불량품 배출 위치
PLACE_TOLERANCE = 0.05                     # 이보다 벗어나면 오배치로 본다 [m]
JUMP_LIMIT_DEG = 30.0                      # 한 스텝에 허용하는 관절 변화 [deg]


def reject_box(stage, box_path, index):
    """
    놓치거나 잘못 놓인 박스를 작업 영역 밖으로 치운다.

    프림을 지우면 물리 시뮬이 불안정해지므로 옮기기만 한다.
    치우지 않으면 픽 존에 남아 다음 스캔이 두 물체를 하나로 인식하고,
    그 뒤 모든 사이클이 연쇄적으로 망가진다.
    """
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(box_path)
    if not prim.IsValid():
        return False

    pos = REJECT_POS + np.array([0.12 * index, 0.0, 0.0])
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            if op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
                op.Set(Gf.Vec3d(*(float(v) for v in pos)))
            else:
                op.Set(Gf.Vec3f(*(float(v) for v in pos)))
            return True
    return False


# ─────────────────────────────────────────────────────────────
# 결과 집계
# ─────────────────────────────────────────────────────────────
class Report:
    def __init__(self):
        self.planned = []     # (box_idx, placement, world_target)
        self.perceived = []
        self.pack_failed = []
        self.grip_failed = []
        self.grip_lost = []      # 운반 중 놓친 박스
        self.misplaced = []      # 계획에서 크게 벗어나 안착한 박스
        self.ik_failed = 0

    def summarize(self, packer, stage):
        section("결과")

        n_seen = len(self.perceived)
        n_planned = len(self.planned)
        print(f"   인식한 박스        {n_seen}")
        print(f"   배치 결정 성공     {n_planned}"
              f"   실패(자리없음) {len(self.pack_failed)}")
        print(f"   흡착 실패          {len(self.grip_failed)}"
              f"   운반 중 놓침 {len(self.grip_lost)}"
              f"   오배치 {len(self.misplaced)}")
        print(f"   IK 실패 step       {self.ik_failed}")
        print()
        print(f"   최종 적재 높이     {packer.current_height*1000:6.1f} mm"
              f"  / 한계 {C.PALLET_MAX_STACK_H*1000:.0f} mm")
        print(f"   부피 활용률        {packer.volume_utilization()*100:6.1f} %"
              f"  (팔레트 전체 기준)")
        print(f"   부피 활용률        {packer.occupied_utilization()*100:6.1f} %"
              f"  (쌓인 높이까지)")

        # 계획 대비 실제 — 알고리즘이 아니라 로봇+물리가 잘 따라줬는지 본다
        if not self.planned:
            return

        errors = []
        per_box = []
        for box_path, placement, target in self.planned:
            prim = stage.GetPrimAtPath(box_path)
            if not prim.IsValid():
                continue
            from pxr import UsdGeom
            mat = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())).T
            actual_center = mat[:3, 3]
            planned_center = target.copy()
            planned_center[2] -= placement.h / 2.0
            errors.append(np.linalg.norm(actual_center - planned_center))
            per_box.append((prim.GetName(), planned_center, actual_center))

        if per_box:
            print()
            print("   박스별 계획 -> 실제")
            for name, plan_c, act_c in per_box:
                d = (act_c - plan_c) * 1000.0
                print(f"      {name:10s} 계획 {vec(plan_c)}  실제 {vec(act_c)}"
                      f"   차이 [{d[0]:+7.1f} {d[1]:+7.1f} {d[2]:+7.1f}] mm")

        if errors:
            errors = np.array(errors) * 1000.0
            print()
            print(f"   계획 대비 실제 위치 오차 ({len(errors)}개)")
            print(f"      평균 {errors.mean():5.1f} mm"
                  f"   중앙 {np.median(errors):5.1f} mm"
                  f"   최대 {errors.max():5.1f} mm")
            settled = int((errors < 20.0).sum())
            print(f"      20 mm 이내 안착  {settled}/{len(errors)}"
                  f"  ({settled/len(errors)*100:.0f} %)")


# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
def main():
    if not os.path.exists(C.USD_PATH):
        raise FileNotFoundError(f"로봇 USD 를 찾지 못했습니다: {C.USD_PATH}")

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()

    section("SCENE")

    add_reference_to_stage(usd_path=C.USD_PATH, prim_path=C.ROBOT_PRIM_PATH)
    if C.PEDESTAL_H > 0:
        from pxr import Gf as _Gf, UsdGeom as _UsdGeom
        _xf = _UsdGeom.Xformable(stage.GetPrimAtPath(C.ROBOT_PRIM_PATH))
        _xf.ClearXformOpOrder()
        _xf.AddTranslateOp().Set(_Gf.Vec3d(*(float(v) for v in C.ROBOT_BASE_POS)))
    scene_builder.build_lights(stage)
    scene_builder.build_pedestal(stage)
    scene_builder.build_conveyor(stage)
    scene_builder.build_pallet(stage)
    print(f"   preset       {C.PRESET}")
    if C.PEDESTAL_H > 0:
        print(f"   pedestal     {C.PEDESTAL_H*1000:.0f} mm"
              f"   -> 로봇 베이스 z {C.ROBOT_BASE_POS[2]:.2f} m")
    print(f"   robot        {C.ROBOT_PRIM_PATH}")
    print(f"   conveyor     center {vec(C.CONVEYOR_CENTER)}"
          f"  top z {C.CONVEYOR_TOP_Z:.3f}")
    print(f"   pallet       {C.PALLET_SIZE[0]*1000:.0f} x {C.PALLET_SIZE[1]*1000:.0f} mm"
          f"  center {vec(C.PALLET_CENTER_XY)}  deck z {C.PALLET_DECK_Z:.3f}"
          f"  max stack {C.PALLET_MAX_STACK_H*1000:.0f} mm")
    print(f"   pallet asset {C.PALLET_ASSET}"
          f"  ({'Isaac pallet.usd' if C.PALLET_USD else '절차적 생성'})")
    corners = [C.PALLET_ORIGIN_XY + np.array([dx, dy])
               for dx in (0.0, C.PALLET_SIZE[0]) for dy in (0.0, C.PALLET_SIZE[1])]
    reach = [float(np.linalg.norm(c - C.ROBOT_BASE_XY)) for c in corners]
    print(f"   reach        pallet {min(reach):.3f}~{max(reach):.3f} m"
          f"   pick {np.linalg.norm(C.PICK_XY - C.ROBOT_BASE_XY):.3f} m"
          f"   ({C.ROBOT_NAME} {C.ROBOT_REACH:.3f} m)")
    print(f"   suction      {C.GRIP_COAXIAL_FORCE_LIMIT:.0f} N"
          f" ({C.GRIP_COAXIAL_FORCE_LIMIT/9.81:.0f} kgf)"
          f"   최대 박스 {max(C.BOX_MASS.values()) if C.BOX_MASS else 0:.0f} kg"
          f" = {max(C.BOX_MASS.values())*9.81 if C.BOX_MASS else 0:.0f} N")
    print(f"   truck        {C.TRUCK_BED[0]*1000:.0f} x {C.TRUCK_BED[1]*1000:.0f} mm"
          f"  <- pallet {C.PALLET_LAYOUT[0]} x {C.PALLET_LAYOUT[1]}"
          f" = {C.PALLET_LAYOUT[0]*C.PALLET_LAYOUT[1]} 개")
    for name, dim in (C.BOX_SPECS.items() if C.BOX_MODE == "spec" else []):
        per_layer = max(
            int(C.PALLET_SIZE[0] // a) * int(C.PALLET_SIZE[1] // b)
            for a, b in ((dim[0], dim[1]), (dim[1], dim[0]))
        )
        layers = int(C.PALLET_MAX_STACK_H // dim[2])
        print(f"   box {C.BOX_GRADE.get(name,''):2s}{name:4s}   "
              f"{dim[0]*1000:3.0f} x {dim[1]*1000:3.0f} x {dim[2]*1000:3.0f} mm"
              f"  {C.BOX_MASS[name]:4.1f} kg  {C.BOX_RATIO[name]*100:3.0f}%"
              f"   층당 {per_layer:2d} x {layers:2d}층 = {per_layer*layers:3d}개")

    ee_path = find_prim_path(C.ROBOT_PRIM_PATH, C.EE_LINK_NAME)
    if ee_path is None:
        raise RuntimeError(f"'{C.EE_LINK_NAME}' 을 {C.ROBOT_PRIM_PATH} 아래에서 찾지 못했습니다")
    print(f"   flange       {ee_path}")

    n_drives = configure_drives(stage)
    print(f"   drives       {n_drives} joints  stiffness {C.DRIVE_STIFFNESS:.0e}"
          f"  damping {C.DRIVE_DAMPING:.0e}  maxEffort {C.DRIVE_MAX_FORCE:.0e}")

    section("SUCTION")
    gripper_path = suction.build_suction(stage, ee_path)
    print(f"   gripper      {gripper_path}")
    print(f"   cup          r {C.SUCTION_CUP_RADIUS*1000:.0f} mm"
          f"  mount z {C.SUCTION_MOUNT_Z*1000:.0f} mm")
    print(f"   tcp offset   {vec(C.TCP_OFFSET)}")
    print(f"   grip dist    {C.GRIP_MAX_DISTANCE*1000:.0f} mm"
          f"   coaxial {C.GRIP_COAXIAL_FORCE_LIMIT:.0f} N"
          f"   shear {C.GRIP_SHEAR_FORCE_LIMIT:.0f} N")

    camera = None
    if C.PERCEPTION_MODE == "camera" or C.PERCEPTION_COMPARE:
        from isaacsim.sensors.camera import Camera

        scene_builder.build_scan_camera(stage)
        camera = Camera(prim_path=C.CAM_PATH, resolution=C.CAM_RESOLUTION)

    robot = world.scene.add(
        SingleManipulator(
            prim_path=C.ROBOT_PRIM_PATH,
            name="m0609",
            end_effector_prim_path=ee_path,
            gripper=None,          # 흡착은 관절이 아니라 SurfaceGripper 로 따로 제어
        )
    )

    world.reset()
    robot.initialize()
    apply_gains(robot)
    set_ready_pose(robot)

    if camera is not None:
        camera.initialize()
        camera.add_distance_to_image_plane_to_frame()

    for _ in range(30):
        world.step(render=True)

    section("SOLVER")
    lula_solver, ik_solver = create_ik_solver(robot)
    print(f"   controlled   {', '.join(lula_solver.get_joint_names())}")

    suction_gripper = suction.SuctionGripper(gripper_path)

    packer = HeightmapPacker(
        size_xy=C.PALLET_SIZE,
        max_height=C.PALLET_MAX_STACK_H,
        cell=C.PACK_CELL,
        yaws_deg=C.PACK_YAWS_DEG,
        min_support_ratio=C.PACK_MIN_SUPPORT_RATIO,
        support_tol=C.PACK_SUPPORT_TOL,
        flatness_weight=C.PACK_FLATNESS_WEIGHT,
        wall_margin=C.PACK_WALL_MARGIN,
        enforce_load_order=C.PACK_ENFORCE_LOAD_ORDER,
        descent_halo=C.PACK_DESCENT_HALO,
    )
    print(f"   heightmap    {packer.nx} x {packer.ny} cells"
          f"  ({C.PACK_CELL*1000:.0f} mm)")
    print(f"   load order   {'무거운 것 아래 강제' if C.PACK_ENFORCE_LOAD_ORDER else '제약 없음'}")
    print(f"   clearance    박스간 {C.PACK_BOX_CLEARANCE*1000:.0f} mm"
          f"   하강통로 {C.PACK_DESCENT_HALO*1000:.0f} mm")

    gt_perception = perception.GroundTruthPerception(stage)
    active_perception = perception.build(
        C.PERCEPTION_MODE, stage=stage, camera=camera
    )
    print(f"   perception   {C.PERCEPTION_MODE}"
          f"{'  (+비교 모드)' if C.PERCEPTION_COMPARE else ''}")

    section("RUN")
    if getattr(C, "AUTO_PLAY", False):
        world.play()
        print(f"   자동 시작.  박스 {C.N_BOXES} 개를 처리합니다.\n")
    else:
        print(f"   Viewport 에서 Play 를 누르세요.  박스 {C.N_BOXES} 개를 처리합니다.\n")

    fsm = CycleFSM(robot)
    report = Report()
    rng = np.random.default_rng(C.RANDOM_SEED)

    was_playing = False
    _prev_q = None
    box_index = 0
    current_box = None
    finished = False

    while simulation_app.is_running():
        try:
            world.step(render=True)
            is_playing = world.is_playing()
        except AttributeError:
            # 창을 닫으면 World 가 먼저 해제되는데 is_running() 은 한 프레임
            # 더 True 를 돌려준다. 그 사이 step() 을 부르면 터진다.
            print("\n   앱 종료 중 — 루프를 정리합니다.")
            break

        # ── Play 시작 / 재시작 ──────────────────────────────
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            apply_gains(robot)
            set_ready_pose(robot)

            # Play 는 하드 리셋이라 카메라 어노테이터가 떨어진다.
            # 로봇과 마찬가지로 매 Play 마다 다시 붙여야 depth 가 나온다.
            if camera is not None:
                camera.initialize()
                camera.add_distance_to_image_plane_to_frame()
                for _ in range(5):
                    world.step(render=True)

            scene_builder.clear_boxes(stage)
            packer.reset()
            report = Report()
            rng = np.random.default_rng(C.RANDOM_SEED)

            box_index = 0
            current_box = None
            finished = False
            suction_gripper.open()
            print("   ▶ 시작\n")

        was_playing = is_playing
        if not is_playing or finished:
            continue

        # ── 다음 박스 투입 + 인식 + 배치 결정 ───────────────
        if current_box is None:
            if box_index >= C.N_BOXES:
                finished = True
                report.summarize(packer, stage)
                if getattr(C, "AUTO_PLAY", False):
                    print("\n   ■ 완료\n")
                    break
                print("\n   ■ 완료 — Stop 후 다시 Play 하면 재실행합니다.\n")
                continue

            box_path, true_size, true_yaw, mass, spec_name = scene_builder.spawn_box(
                stage, box_index, rng
            )

            # 물리적으로 안정되기를 잠깐 기다린다
            for _ in range(20):
                world.step(render=True)

            # 뷰포트에서 카메라 시점을 돌렸을 수 있으므로 스캔 직전에 고정
            if camera is not None:
                scene_builder.lock_scan_camera(stage)
                world.step(render=True)

            if os.environ.get("PL_DEBUG_ROI"):
                debug_roi_contents(stage, f" box{box_index:02d}")

            obs = active_perception.observe(box_path)
            if obs is None and camera is not None:
                # 첫 프레임은 depth 가 아직 안 올라와 있을 수 있다. 한 번 더 준다.
                for _ in range(10):
                    world.step(render=True)
                obs = active_perception.observe(box_path)

            if obs is None:
                reason = ""
                if camera is not None:
                    depth = camera.get_depth()
                    if depth is None:
                        reason = " (depth 없음 — 카메라 어노테이터 미부착)"
                    else:
                        d = np.asarray(depth)
                        reason = (f" (depth {d.shape} 유효 {np.isfinite(d).sum()}px"
                                  f" — ROI 안에 점군이 부족)")
                print(f"   [{box_index:02d}] 인식 실패 — 건너뜀{reason}")
                box_index += 1
                continue

            report.perceived.append(obs)

            print(f"   [{box_index:02d}] {spec_name}  인식  {obs}  {mass:.1f} kg")
            if obs.source == "camera" and obs.fill < C.CAM_MIN_FILL:
                print(f"        ⚠ 채움률 {obs.fill*100:.0f}% — 피팅한 사각형이 많이 비었습니다."
                      f" 두 물체를 하나로 인식했을 수 있습니다.")

            if C.PERCEPTION_COMPARE and camera is not None:
                gt_obs = gt_perception.observe(box_path)
                cam_obs = perception.CameraPerception(camera).observe()
                print(perception.compare(gt_obs, cam_obs))

            # 박스 발자국을 여유만큼 부풀려 자리를 잡는다. 중심은 그대로이므로
            # 실제 놓는 좌표는 변하지 않고, 이웃과의 간격만 생긴다.
            pack_size = obs.size.copy()
            pack_size[:2] += C.PACK_BOX_CLEARANCE
            placement = packer.place(pack_size, mass)
            if placement is None:
                print(f"   [{box_index:02d}] 배치 실패 — 팔레트에 자리 없음")
                report.pack_failed.append(box_index)
                box_index += 1
                continue

            place_top_world = packer.local_to_world(
                placement.top_center_local, C.PALLET_ORIGIN_XY, C.PALLET_DECK_Z
            )

            print(f"        배치  local {vec(placement.top_center_local)}"
                  f" -> world {vec(place_top_world)}"
                  f"  yaw {placement.yaw_deg:+.0f}°"
                  f"  {mass:.1f}kg  지지율 {placement.support_ratio*100:.0f}%"
                  f"  평탄도 {placement.flatness*1000:.1f} mm")

            fsm.plan(
                pick_top_world=obs.top_center,
                pick_yaw_deg=obs.yaw_deg,
                place_top_world=place_top_world,
                place_yaw_deg=placement.yaw_deg,
                stack_top_world_z=C.PALLET_DECK_Z + packer.current_height,
                box_h=float(obs.size[2]),
            )
            current_box = (box_path, placement, place_top_world)
            continue

        # ── 사이클 실행 ─────────────────────────────────────
        target_tcp = fsm.current_target()
        target_quat = make_target_quat(fsm.current_yaw())
        flange_target = tcp_to_flange(target_tcp, target_quat)

        q_sol, solved = solve_ik(lula_solver, robot, flange_target, target_quat)
        if solved:
            from isaacsim.core.utils.types import ArticulationAction
            q_full = np.array(robot.get_joint_positions(), dtype=float)
            q_full[:6] = q_sol
            action = ArticulationAction(joint_positions=q_full)
            robot.apply_action(action)
        else:
            report.ik_failed += 1

        suction_gripper.forward(fsm.suction)

        if os.environ.get("PL_DEBUG_DESC") and current_box is not None \
                and fsm.state == fsm.DESCEND_PLACE and fsm.step % 15 == 0:
            from pxr import UsdGeom as _UG
            _bp = stage.GetPrimAtPath(current_box[0])
            if _bp.IsValid():
                _m = np.array(_UG.Xformable(_bp).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())).T
                _bc = _m[:3, 3]
                # 회전행렬 열에 스케일이 섞여 있으므로 정규화해야 한다
                _R = _m[:3, :3] / np.linalg.norm(_m[:3, :3], axis=0)
                _h = float(current_box[1].h)
                _btop = _bc + _R @ np.array([0.0, 0.0, _h / 2.0])
                _tcp = get_tcp_pose(robot)
                _d = np.linalg.norm(_btop - _tcp)
                _tilt = np.degrees(np.arccos(np.clip(
                    abs(np.dot(_R[:, 2], np.array([0.0, 0.0, 1.0]))), -1, 1)))
                _q = np.degrees(robot.get_joint_positions()[:6])
                _dq = _q - _prev_q if _prev_q is not None else np.zeros(6)
                _prev_q = _q
                print(f"      [DESC {fsm.step:3d}] 거리 {_d*1000:5.1f}mm"
                      f"  기울기 {_tilt:4.1f}deg  흡착 {suction_gripper.status:7s}"
                      f"  관절 {np.round(_q,0)}  최대변화 {np.max(np.abs(_dq)):5.2f}deg")

        if os.environ.get("PL_DEBUG_GRIP") and fsm.step == 0 \
                and fsm.state == fsm.DESCEND_PICK:
            q = robot.get_joint_positions()
            print(f"      dof {robot.num_dof}  names {robot.dof_names}")
            print(f"      현재 관절(deg) {np.round(np.degrees(q[:6]),1)}")
            if solved:
                print(f"      IK 목표(deg)   {np.round(np.degrees(q_sol),1)}")

        if os.environ.get("PL_DEBUG_GRIP"):
            if fsm.state in (fsm.DESCEND_PICK, fsm.GRIP, fsm.LIFT) and fsm.step % 30 == 0:
                tcp = get_tcp_pose(robot)
                ee_pos, ee_quat = robot.end_effector.get_world_pose()
                gap = tcp[2] - fsm.waypoints[fsm.DESCEND_PICK][2]
                print(f"      [{fsm.NAMES[fsm.state]:10s} {fsm.step:3d}]"
                      f" 목표 {vec(target_tcp)}  TCP {vec(tcp)}"
                      f"  박스윗면 {fsm.waypoints[fsm.DESCEND_PICK][2]:.3f}"
                      f"  틈 {gap*1000:+6.1f}mm"
                      f"  IK {'O' if solved else 'X'}"
                      f"  흡착 {suction_gripper.status}"
                      f"  대상 {suction_gripper.gripped_object()}")

        # 흡착 결과 확인 — LIFT 진입 시점에 붙었는지 본다
        if fsm.state == fsm.LIFT and fsm.step == 1:
            if not suction_gripper.is_gripping:
                print(f"        흡착 실패 (status={suction_gripper.status})")
                report.grip_failed.append(box_index)

        # 운반 구간(LIFT ~ 내려놓기 직전) 내내 붙어 있는지 계속 본다.
        # 여기서 놓친 박스를 방치하면 픽 존에 남아 다음 스캔을 오염시킨다.
        if fsm.LIFT <= fsm.state < fsm.RELEASE and not suction_gripper.is_gripping:
            box_path, placement, _ = current_box
            _tcp = get_tcp_pose(robot)
            print(f"        운반 중 놓침 ({fsm.NAMES[fsm.state]} step {fsm.step}) — 박스 배출"
                  f"   TCP {vec(_tcp)}  목표지점 {vec(current_box[2])}"
                  f"  남은높이 {(_tcp[2]-current_box[2][2])*1000:+.0f}mm")
            report.grip_lost.append(box_index)
            packer.undo_last()
            reject_box(stage, box_path, box_index)
            box_index += 1
            current_box = None
            fsm.state = fsm.DONE
            continue

        fsm.advance(verbose=False)

        if fsm.done:
            box_path, placement, place_top_world = current_box

            # 안착 위치 검증 — 계획에서 크게 벗어났으면 하이트맵이 거짓이 된다.
            # 그대로 두면 이후 배치가 전부 어긋나므로 되돌리고 배출한다.
            from pxr import UsdGeom as _UsdGeom
            prim = stage.GetPrimAtPath(box_path)
            planned_center = place_top_world.copy()
            planned_center[2] -= placement.h / 2.0
            err = float("inf")
            if prim.IsValid():
                mat = np.array(_UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())).T
                err = float(np.linalg.norm(mat[:3, 3] - planned_center))

            if err > PLACE_TOLERANCE:
                act = mat[:3, 3] if prim.IsValid() else np.zeros(3)
                print(f"        오배치 {err*1000:.0f} mm — 되돌리고 배출"
                      f"   계획 {vec(planned_center)}  실제 {vec(act)}"
                      f"   차이 [{(act[0]-planned_center[0])*1000:+.0f}"
                      f" {(act[1]-planned_center[1])*1000:+.0f}"
                      f" {(act[2]-planned_center[2])*1000:+.0f}] mm\n")
                report.misplaced.append(box_index)
                packer.undo_last()
                reject_box(stage, box_path, box_index)
            else:
                report.planned.append((box_path, placement, place_top_world))
                print(f"        완료  오차 {err*1000:4.1f} mm"
                      f"  적재높이 {packer.current_height*1000:5.1f} mm"
                      f"  활용률 {packer.occupied_utilization()*100:4.1f} %\n")

            box_index += 1
            current_box = None

    simulation_app.close()


if __name__ == "__main__":
    main()
