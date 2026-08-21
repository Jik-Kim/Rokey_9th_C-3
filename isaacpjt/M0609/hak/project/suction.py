"""
흡착 그리퍼 (Isaac Sim 5.1 SurfaceGripper)

왜 평행 그리퍼가 아니라 흡착인가:
  1) RG2 스트로크가 약 110 mm 라 100 mm 이상 박스는 옆에서 잡지 못한다.
  2) 더 중요한 이유 — 3D 패킹은 박스를 서로 밀착시켜 쌓는 것이 전부인데,
     평행 그리퍼는 손가락 두께만큼 옆 공간을 비워야 해서 밀착 적재 자체가
     불가능하다. 위에서 빨면 간격 0 으로 붙는다.
  실물 대응: OnRobot VGC10 / VG10 (RG2 와 같은 OnRobot 퀵체인저)

5.1 에서 API 가 바뀌었다. 예전
    SurfaceGripper(translate=..., direction="x", grip_threshold=...)
방식 예제는 그대로 쓰면 동작하지 않는다. 지금 구조는

    IsaacSurfaceGripper prim
      └ isaac:attachmentPoints -> [D6 PhysicsJoint (IsaacAttachmentPointAPI)]
                                     body0 = 흡착하는 쪽(로봇 링크)
                                     body1 = 런타임에 빨린 물체로 채워짐

파라미터는 data/SurfaceGripper_gantry.usda 의 값을 그대로 따랐다.
"""

from __future__ import annotations

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema

import config as C

FLT_MAX = 3.4028235e38


def _set(prim, name, value, type_name):
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, False)
    attr.Set(value)
    return attr


def build_suction(stage, link6_path: str) -> str:
    """
    link_6 에 흡착판과 SurfaceGripper 를 만들어 붙인다.

    흡착판은 link_6 의 자식 지오메트리로 넣는다 — 별도 rigid body + FixedJoint 를
    쓰지 않으므로 아티큘레이션이 하나로 유지되고 IK 가 그대로 먹는다.

    Returns: 생성된 SurfaceGripper prim 경로
    """
    # SurfaceGripper / 부착점 프림도 link_6 밑에 만든다.
    # /World/Suction 처럼 월드 최상위에 두면 프림 자체는 원점(0,0,0)에
    # 남아, 뷰포트에서 기즈모가 로봇과 동떨어져 떠 있는 것처럼 보인다.
    # (흡착 동작 자체는 body0 관계로 이뤄지므로 기능엔 문제가 없었다)
    gripper_path = f"{link6_path}/SurfaceGripper"
    joints_scope = f"{link6_path}/AttachmentPoints"

    # ── 그리퍼 몸통 (플랜지 면 ~ 흡착판) ────────────────────
    # 플랜지 끝면이 link_6 로컬 z=0 이고 흡착판은 z=SUCTION_MOUNT_Z 에서
    # 시작한다. 그 사이가 비어 있으면 흡착판만 공중에 떠 있는 것처럼 보인다.
    # 실물로 치면 퀵체인저와 진공 발생기가 들어가는 자리다.
    if C.SUCTION_MOUNT_Z > 0:
        body_path = f"{link6_path}/suction_body"
        body = UsdGeom.Cylinder.Define(stage, body_path)
        body.CreateAxisAttr("Z")
        body.CreateRadiusAttr(float(C.SUCTION_BODY_RADIUS))
        body.CreateHeightAttr(float(C.SUCTION_MOUNT_Z))
        body.CreateExtentAttr([
            Gf.Vec3f(-C.SUCTION_BODY_RADIUS, -C.SUCTION_BODY_RADIUS,
                     -C.SUCTION_MOUNT_Z / 2.0),
            Gf.Vec3f(C.SUCTION_BODY_RADIUS, C.SUCTION_BODY_RADIUS,
                     C.SUCTION_MOUNT_Z / 2.0),
        ])
        body.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.36, 0.40)])
        UsdGeom.Xformable(body).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, C.SUCTION_MOUNT_Z / 2.0))
        # 흡착판과 같은 이유로 충돌은 끈다 (접근 중 박스를 밀어낸다).
        UsdPhysics.CollisionAPI.Apply(body.GetPrim()).CreateCollisionEnabledAttr(False)

    # ── 흡착판 지오메트리 (link_6 자식) ─────────────────────
    cup_path = f"{link6_path}/suction_cup"
    cup = UsdGeom.Cylinder.Define(stage, cup_path)
    cup.CreateAxisAttr("Z")
    cup.CreateRadiusAttr(float(C.SUCTION_CUP_RADIUS))
    cup.CreateHeightAttr(float(C.SUCTION_CUP_THICKNESS))
    cup.CreateExtentAttr([
        Gf.Vec3f(-C.SUCTION_CUP_RADIUS, -C.SUCTION_CUP_RADIUS,
                 -C.SUCTION_CUP_THICKNESS / 2.0),
        Gf.Vec3f(C.SUCTION_CUP_RADIUS, C.SUCTION_CUP_RADIUS,
                 C.SUCTION_CUP_THICKNESS / 2.0),
    ])
    cup.CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.15, 0.18)])

    cup_z = C.SUCTION_MOUNT_Z + C.SUCTION_CUP_THICKNESS / 2.0
    UsdGeom.Xformable(cup).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, cup_z))

    # 흡착판에 충돌체를 주면 접근 중 박스를 밀어버린다.
    # 정지 높이는 모션이 제어하므로 시각용으로만 둔다.
    UsdPhysics.CollisionAPI.Apply(cup.GetPrim()).CreateCollisionEnabledAttr(False)

    # ── SurfaceGripper prim ────────────────────────────────
    gripper_prim = robot_schema.CreateSurfaceGripper(stage, gripper_path)

    _set(gripper_prim, robot_schema.Attributes.MAX_GRIP_DISTANCE.name,
         float(C.GRIP_MAX_DISTANCE), Sdf.ValueTypeNames.Float)
    _set(gripper_prim, robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name,
         float(C.GRIP_COAXIAL_FORCE_LIMIT), Sdf.ValueTypeNames.Float)
    _set(gripper_prim, robot_schema.Attributes.SHEAR_FORCE_LIMIT.name,
         float(C.GRIP_SHEAR_FORCE_LIMIT), Sdf.ValueTypeNames.Float)
    _set(gripper_prim, robot_schema.Attributes.RETRY_INTERVAL.name,
         float(C.GRIP_RETRY_INTERVAL), Sdf.ValueTypeNames.Float)
    _set(gripper_prim, robot_schema.Attributes.STATUS.name,
         "Open", Sdf.ValueTypeNames.Token)

    # ── 부착점 = D6 조인트 ──────────────────────────────────
    UsdGeom.Scope.Define(stage, joints_scope)
    joint_path = f"{joints_scope}/D6Joint"
    joint = UsdPhysics.Joint.Define(stage, joint_path)
    joint_prim = joint.GetPrim()

    # body0 = 빠는 쪽. body1 은 런타임에 빨린 물체로 채워진다.
    joint.CreateBody0Rel().SetTargets([Sdf.Path(link6_path)])
    joint.CreateBody1Rel().SetTargets([])

    # 흡착면 위치/자세 (link_6 로컬). +Z 가 흡착 방향.
    joint.CreateLocalPos0Attr().Set(
        Gf.Vec3f(0.0, 0.0, float(C.SUCTION_MOUNT_Z + C.SUCTION_CUP_THICKNESS))
    )
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    joint.CreateBreakForceAttr().Set(FLT_MAX)
    joint.CreateBreakTorqueAttr().Set(FLT_MAX)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateJointEnabledAttr().Set(True)

    # 병진: X/Y 잠금, Z 는 흡착판이 살짝 눌리도록 0~10 mm
    for axis in ("transX", "transY"):
        limit = UsdPhysics.LimitAPI.Apply(joint_prim, axis)
        limit.CreateLowAttr().Set(1.0)     # low > high = 완전 잠금
        limit.CreateHighAttr().Set(-1.0)
        PhysxSchema.PhysxLimitAPI.Apply(joint_prim, axis)

    limit_z = UsdPhysics.LimitAPI.Apply(joint_prim, "transZ")
    limit_z.CreateLowAttr().Set(0.0)
    limit_z.CreateHighAttr().Set(0.01)

    # 회전: 흡착판 고무가 휘는 만큼만 (±3 rad 은 샘플 값, 드라이브로 잡는다)
    for axis in ("rotX", "rotY", "rotZ"):
        limit = UsdPhysics.LimitAPI.Apply(joint_prim, axis)
        limit.CreateLowAttr().Set(-3.0)
        limit.CreateHighAttr().Set(3.0)

    # 드라이브 — 프리셋에서 정한다. 무거운 박스일수록 강성/감쇠가 커야
    # 매달린 박스가 흔들리다 떨어지지 않는다.
    for axis, (stiffness, damping) in C.SUCTION_DRIVE.items():
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, axis)
        drive.CreateStiffnessAttr().Set(stiffness)
        if damping:
            drive.CreateDampingAttr().Set(damping)

    # IsaacAttachmentPointAPI — robot_schema 의 헬퍼는 상수 이름을 잘못 쓰므로
    # (Classes.ATTACHMENT_POINT_API.name 이 .value 대신) 직접 적용한다.
    joint_prim.AddAppliedSchema(robot_schema.Classes.ATTACHMENT_POINT_API.value)
    _set(joint_prim, robot_schema.Attributes.FORWARD_AXIS.name,
         "Z", Sdf.ValueTypeNames.Token)
    _set(joint_prim, robot_schema.Attributes.CLEARANCE_OFFSET.name,
         float(C.GRIP_CLEARANCE_OFFSET), Sdf.ValueTypeNames.Float)

    gripper_prim.GetRelationship(
        robot_schema.Relations.ATTACHMENT_POINTS.name
    ).SetTargets([Sdf.Path(joint_path)])

    return gripper_path


class SuctionGripper:
    """
    close/open + 실제로 붙었는지 조회.

    ParallelGripper 와 달리 관절 목표를 내보내지 않으므로 로봇 액션과 독립이다.
    -> FSM 에서는 robot.apply_action() 과 나란히 호출하면 된다.
    """

    def __init__(self, gripper_path: str = C.SUCTION_GRIPPER_PATH):
        import isaacsim.robot.surface_gripper._surface_gripper as sg

        self._sg = sg
        self._iface = sg.acquire_surface_gripper_interface()
        self._path = gripper_path
        self._commanded = "open"

    # ── 명령 ────────────────────────────────────────────────
    def close(self) -> None:
        self._commanded = "close"
        self._iface.close_gripper(self._path)

    def open(self) -> None:
        self._commanded = "open"
        self._iface.open_gripper(self._path)

    def forward(self, action: str) -> None:
        """FSM 이 매 step 내리는 명령. 이미 그 상태면 무시한다."""
        if action == "close":
            if self.status != "Closed":
                self.close()
        elif action == "open":
            if self.status != "Open":
                self.open()
        else:
            raise ValueError(f"action 은 open|close 만 가능: {action!r}")

    # ── 상태 ────────────────────────────────────────────────
    @property
    def status(self) -> str:
        """"Open" | "Closing" | "Closed"

        인터페이스는 문자열이 아니라 GripperStatus enum 을 돌려주고
        str() 하면 "GripperStatus.Closed" 가 된다. 뒤쪽만 떼어 쓴다.
        """
        return str(self._iface.get_gripper_status(self._path)).rsplit(".", 1)[-1]

    @property
    def is_gripping(self) -> bool:
        return self.status == "Closed"

    def gripped_object(self) -> str | None:
        try:
            objects = self._iface.get_gripped_objects_batch([self._path])
        except Exception:
            return None
        if not objects:
            return None
        first = objects[0]
        if isinstance(first, (list, tuple)):
            first = first[0] if first else None
        return str(first) if first else None


def approach_pose_for(top_center_world, clearance: float) -> np.ndarray:
    """흡착 대상 윗면 위 clearance 만큼 띄운 TCP 목표."""
    p = np.asarray(top_center_world, dtype=float).copy()
    p[2] += clearance
    return p
