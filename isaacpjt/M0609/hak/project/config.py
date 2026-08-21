"""
1차 목표 (한 라인) 공통 설정

여기 값만 고치면 씬/알고리즘/로봇 동작이 전부 따라갑니다.
길이 단위는 별도 표기가 없으면 meter.
"""

import numpy as np
from pathlib import Path


# ─────────────────────────────────────────────────────────────
# 실행 모드
# ─────────────────────────────────────────────────────────────
# "gt"     : 박스 USD prim에서 bounding box를 직접 읽어 치수/자세 획득 (빠름, 기본)
# "camera" : 카메라 depth -> 포인트클라우드 -> 평면 분할 + OBB 피팅 (진짜 인식)
PERCEPTION_MODE = "gt"

# True 로 두면 매 박스마다 두 방식을 모두 돌려 오차를 출력한다.
# 카메라 구현을 튜닝할 때 켜고, 데모할 때는 끈다.
PERCEPTION_COMPARE = False

HEADLESS = False

# 한 번의 Play에서 처리할 박스 개수
N_BOXES = 20


# ─────────────────────────────────────────────────────────────
# 경로
# ─────────────────────────────────────────────────────────────
THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent          # .../M0609/hak

# 로봇 자산은 프리셋에서 정한다 (아래 ROBOT 섹션).
# M0609: Collected_m0609_camera* 는 둘 다 RG2 + RealSense 가 붙어 있어
#        흡착판을 달 자리가 없다. SubUSDs 안의 파일이 팔 단독 USD.
# H2017: USD 는 있었지만 URDF 가 없어서, USD 의 조인트 로컬 프레임을 읽어
#        urdf_h2017/h2017.urdf 를 직접 만들었다 (FK 오차 0.00mm 검증됨).
M0609_USD = str(M0609_DIR / "Collected_m0609_camera_cube/SubUSDs/m0609_isaac_sim.usd")
M0609_URDF = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESC = str(M0609_DIR / "descriptor/m0609_description.yaml")

H2017_USD = str(M0609_DIR / "doosan-robot2/usd/h2017.usd")
H2017_URDF = str(M0609_DIR / "doosan-robot2/urdf_h2017/h2017.urdf")
H2017_DESC = str(M0609_DIR / "descriptor_h2017/h2017_description.yaml")


# ─────────────────────────────────────────────────────────────
# 로봇
# ─────────────────────────────────────────────────────────────
ROBOT_PRIM_PATH = "/World/robot"
EE_LINK_NAME = "link_6"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# 받침대 위에 올라간다 (h2017). 다른 프리셋은 PEDESTAL_H = 0.
ROBOT_BASE_POS = None      # 아래 파생값 섹션에서 PEDESTAL_H 반영
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])
READY_JOINTS_DEG = None    # 파생값 섹션에서 프리셋별로 설정

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
# DRIVE_MAX_FORCE 는 프리셋에서 정한다. 값이 너무 크면 접촉이 조금만 생겨도
# 박스를 튕겨낸다 (postal 프리셋에서 실제로 1.8m 날아갔다).


# ─────────────────────────────────────────────────────────────
# 흡착 그리퍼 (Isaac Sim 5.1 SurfaceGripper)
# ─────────────────────────────────────────────────────────────
SUCTION_ROOT = "/World/Suction"
SUCTION_CUP_PATH = f"{SUCTION_ROOT}/Cup"          # 흡착판 rigid body
SUCTION_GRIPPER_PATH = f"{SUCTION_ROOT}/SurfaceGripper"
SUCTION_JOINTS_PATH = f"{SUCTION_ROOT}/AttachmentPoints"

# link_6 플랜지 기준 흡착판 장착 오프셋 (+Z가 툴 진행 방향)
SUCTION_MOUNT_Z = 0.08
SUCTION_CUP_RADIUS = 0.035
SUCTION_CUP_THICKNESS = 0.012

# 플랜지와 흡착판 사이를 채우는 그리퍼 몸통 (퀵체인저 + 진공 발생기).
# 이게 없으면 흡착판만 플랜지에서 SUCTION_MOUNT_Z 만큼 공중에 떠 보인다.
# 시각용이라 충돌은 끈다 — TCP_OFFSET 과 모션 값에는 영향이 없다.
SUCTION_BODY_RADIUS = 0.045

# 흡착판 아랫면까지의 거리 = TCP 오프셋
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_MOUNT_Z + SUCTION_CUP_THICKNESS / 2.0])

# SurfaceGripper 물리 파라미터
GRIP_MAX_DISTANCE = 0.02        # 이 거리 안에 표면이 들어와야 흡착 시도
GRIP_COAXIAL_FORCE_LIMIT = 200.0  # 축방향(수직) 흡착 유지력 [N]
GRIP_SHEAR_FORCE_LIMIT = 100.0    # 전단(옆으로 미끄러짐) 유지력 [N]
GRIP_RETRY_INTERVAL = 0.5       # close 후 흡착 재시도 지속 시간 [s]
GRIP_CLEARANCE_OFFSET = 0.008


# ─────────────────────────────────────────────────────────────
# 프리셋  —  PL_PRESET=demo | postal   (기본 demo)
# ─────────────────────────────────────────────────────────────
#   demo    : 작은 랜덤 박스 + 소형 팔레트.
#             파이프라인 전체가 검증된 설정 (12/12 성공, 안착 오차 중앙 13.5mm).
#             데모/회귀 테스트용 기준선.
#   postal  : 우체국 규격박스 1~2호 실치수 + ISO 600x400 팔레트 (M0609).
#             M0609 리치 한계로 용량이 작다. 참고용으로 남겨둠.
#   h2017   : 확정 사양. H2017 + 받침대 400mm + 1250x800 팔레트 + 3/4/5호.
import os as _os

PRESET = _os.environ.get("PL_PRESET", "h2017")
if PRESET not in ("demo", "postal", "h2017"):
    raise ValueError(f"PL_PRESET 은 demo | postal | h2017 만 가능: {PRESET!r}")


if PRESET == "demo":
    # ── 컨베이어 / 픽 존 ──────────────────────────────────
    CONVEYOR_CENTER = np.array([0.32, 0.34, 0.0])
    CONVEYOR_SIZE = np.array([0.50, 0.22, 0.04])
    PICK_XY = np.array([0.32, 0.34])

    # ── 팔레트 ────────────────────────────────────────────
    PALLET_SIZE = np.array([0.28, 0.22])
    PALLET_CENTER_XY = np.array([0.44, -0.20])
    PALLET_DECK_Z = 0.04
    PALLET_MAX_STACK_H = 0.24

    # ── 박스: 연속 랜덤 ───────────────────────────────────
    BOX_MODE = "random"
    BOX_MIN, BOX_MAX = 0.045, 0.090
    BOX_DENSITY = 120.0                      # kg/m^3
    BOX_SPECS = {}                           # random 모드에서는 안 씀
    BOX_MASS = {}
    BOX_NAMES = []
    BOX_MAX_DIM = BOX_MAX
    BOX_MAX_H = BOX_MAX

    # ── 모션 ──────────────────────────────────────────────
    PLACE_RELEASE_GAP = 0.004
    DRIVE_MAX_FORCE = 1.0e8

elif PRESET == "postal":
    # ── 컨베이어 / 픽 존 ──────────────────────────────────
    CONVEYOR_CENTER = np.array([0.25, 0.48, 0.0])
    CONVEYOR_SIZE = np.array([0.50, 0.30, 0.04])
    PICK_XY = np.array([0.25, 0.48])

    # ── 팔레트: ISO 600x400 쿼터 팔레트 ───────────────────
    #   · M0609 리치      최원거리 762mm (리치 900mm)
    #   · 박스 타일링     1호 4개 · 2호 4개 / 층
    #   · 1톤 트럭        16개 = 2400 x 1600 -> 2500 x 1600 에 딱
    #   · 표준 연계       4개 = EUR1(1200x800), EUR1 4개 = 트럭 만재
    PALLET_SIZE = np.array([0.40, 0.60])
    PALLET_CENTER_XY = np.array([0.50, 0.0])
    PALLET_DECK_Z = 0.04
    # 적재 상한. 2호(150mm)를 들고 이 높이를 넘으려면 TCP 가 0.50m 까지
    # 올라가는데, 팔레트 최원거리 762mm 에서의 도달 한계가 763mm 라 여기가 경계.
    PALLET_MAX_STACK_H = 0.21

    # ── 박스: 우체국 표준 규격 (실치수) ───────────────────
    # 3호(340x250x210, 5.0kg)는 무게는 되지만 높이 때문에 M0609 리치를 벗어난다.
    # M1013(리치 1300)으로 올리면 3호를 넣고 팔레트를 800x600 으로 키우면 된다.
    BOX_MODE = "spec"
    BOX_SPECS = {
        "1호": np.array([0.220, 0.190, 0.090]),
        "2호": np.array([0.270, 0.180, 0.150]),
    }
    # M0609 가반 6kg - 흡착 그리퍼(VGC10 약 1.1kg) = 실사용 4.9kg
    BOX_MASS = {"1호": 1.5, "2호": 3.0}
    BOX_NAMES = list(BOX_SPECS.keys())
    BOX_MIN = float(min(d.min() for d in BOX_SPECS.values()))
    BOX_MAX = float(max(d.max() for d in BOX_SPECS.values()))
    BOX_DENSITY = 280.0
    BOX_MAX_DIM = BOX_MAX
    BOX_MAX_H = float(max(d[2] for d in BOX_SPECS.values()))

    # ── 모션 ──────────────────────────────────────────────
    PLACE_RELEASE_GAP = 0.003
    DRIVE_MAX_FORCE = 5.0e3

else:  # h2017  — 확정 사양 (2026-08-20 사양서)
    # ── 좌표계 ────────────────────────────────────────────
    # 사양서는 파렛트 원점 기준이다.
    #   파렛트 영역   x 0~1200, y 0~800      상면 z = 150
    #   적재 상단     z = 1450  (트럭 내高 1600 - 파렛트 150)
    #   로봇 베이스   (600, -300, 700)  장변 중앙 · 이격 300 · 받침대 700
    # 이 좌표계를 그대로 쓴다 (로봇을 원점에 두지 않는다).
    ROBOT_BASE_XY = np.array([0.600, -0.300])
    PEDESTAL_H = 0.700

    # ── 파렛트 (Isaac pallet.usd = EUR1 1213x802) ─────────
    PALLET_ASSET = _os.environ.get("PL_PALLET", "std")
    if PALLET_ASSET not in ("std", "small"):
        raise ValueError(f"PL_PALLET 은 std | small: {PALLET_ASSET!r}")

    if PALLET_ASSET == "std":
        PALLET_USD = str(M0609_DIR / "assets/pallets/pallet.usd")
        PALLET_SIZE = np.array([1.200, 0.800])   # 장변 x, 단변 y
        PALLET_CENTER_XY = np.array([0.600, 0.400])
    else:
        PALLET_USD = None
        PALLET_SIZE = np.array([0.800, 0.600])   # EUR6 하프
        PALLET_CENTER_XY = np.array([0.600, 0.300])

    PALLET_DECK_Z = 0.150                        # 사양: 상면 z = 150
    PALLET_TOP_Z = 1.450                         # 사양: 적재 상단 z = 1450
    PALLET_MAX_STACK_H = PALLET_TOP_Z - PALLET_DECK_Z   # = 1300 mm

    # ── 컨베이어 / 픽 존 ──────────────────────────────────
    # 파렛트 옆(-x 쪽)에 나란히 둔다. 실제 팔레타이징 셀의 인피드 배치다.
    #
    # 로봇 반대편(y = -1.05)에 두면 픽 방위각이 -90도, 파렛트는 +27~153도라
    # 매 사이클 팔이 150도 넘게 돌아야 하고 그 경로의 방위각 0도 부근에서
    # 흡착이 끊긴다. 옆으로 옮기면 픽 방위각이 155도가 되어 회전각이
    # 128도 이하로 줄고 문제 구간을 아예 지나지 않는다.
    # 트럭 적재함 바닥 지상고 (1톤 윙바디). 측면 윙바디에 컨베이어를 대고
    # 직접 싣는 방식이라 컨베이어 토출 높이 = 트럭 바닥 높이여야 한다.
    # test1.usd 는 이 값으로 라인 전 구간을 평탄화해 뒀다 (flatten_line.py).
    TRUCK_FLOOR_H = 0.900
    CONVEYOR_TOP_H = TRUCK_FLOOR_H
    # 파렛트(x >= 0)와 300mm 이상 띄운다. 붙여 놓으면 파렛트 근처 자리로
    # 내려오는 박스가 컨베이어 상면(900mm) 모서리를 스쳐 흡착이 끊긴다.
    CONVEYOR_CENTER = np.array([-0.600, 0.125, CONVEYOR_TOP_H - 0.04])
    CONVEYOR_SIZE = np.array([0.600, 1.200, 0.04])
    PICK_XY = np.array([-0.600, 0.125])

    # ── 박스: 우체국 규격 3 · 4 · 5호 = 소 · 중 · 대 ──────
    BOX_MODE = "spec"
    BOX_SPECS = {
        "3호": np.array([0.340, 0.250, 0.210]),   # 소
        "4호": np.array([0.410, 0.310, 0.280]),   # 중
        "5호": np.array([0.480, 0.360, 0.340]),   # 대
    }
    BOX_MASS = {"3호": 3.0, "4호": 5.0, "5호": 9.0}      # kg
    BOX_GRADE = {"3호": "소", "4호": "중", "5호": "대"}
    BOX_RATIO = {"3호": 0.50, "4호": 0.30, "5호": 0.20}  # 혼입 비율
    BOX_NAMES = list(BOX_SPECS.keys())
    BOX_MIN = float(min(d.min() for d in BOX_SPECS.values()))
    BOX_MAX = float(max(d.max() for d in BOX_SPECS.values()))
    BOX_DENSITY = 170.0
    BOX_MAX_DIM = BOX_MAX
    BOX_MAX_H = float(max(d[2] for d in BOX_SPECS.values()))

    # ── 로봇 게인 ─────────────────────────────────────────
    # H2017 자중 75~79 kg. M0609 용 1e8 은 솔버가 발산해 클램프된다.
    DRIVE_STIFFNESS = 2.0e6
    DRIVE_DAMPING = 2.0e5
    DRIVE_MAX_FORCE = 2.0e4

    # ── 모션 ──────────────────────────────────────────────
    PLACE_RELEASE_GAP = 0.012
    SUCTION_DRIVE = {
        "transZ": (5.0e5, 5.0e4),
        "rotX":   (5.0e5, 5.0e4),
        "rotY":   (5.0e5, 5.0e4),
        "rotZ":   (5.0e5, 5.0e4),
    }


# ── 프리셋 공통 파생값 ───────────────────────────────────
CONVEYOR_PATH = "/World/Conveyor"
CONVEYOR_TOP_Z = CONVEYOR_CENTER[2] + CONVEYOR_SIZE[2]

# 받침대 (h2017 프리셋만 사용). 로봇 베이스가 이 높이에 놓인다.
PEDESTAL_H = globals().get("PEDESTAL_H", 0.0)
PEDESTAL_PATH = "/World/Pedestal"
ROBOT_BASE_XY = globals().get("ROBOT_BASE_XY", np.array([0.0, 0.0]))
ROBOT_BASE_POS = np.array([float(ROBOT_BASE_XY[0]), float(ROBOT_BASE_XY[1]),
                           float(PEDESTAL_H)])

# 부착 조인트 드라이브 기본값 (Isaac 샘플 값)
if not globals().get("SUCTION_DRIVE"):
    SUCTION_DRIVE = {
        "transZ": (5000.0, 100.0),
        "rotX":   (100.0, 0.0),
        "rotY":   (100.0, 0.0),
        "rotZ":   (10000.0, 0.0),
    }

# ── 프리셋별 로봇 자산 ───────────────────────────────────
if PRESET == "h2017":
    USD_PATH = H2017_USD
    URDF_PATH = H2017_URDF
    DESCRIPTION_PATH = H2017_DESC
    ROBOT_NAME = "H2017"
    ROBOT_REACH = 1.700
    ROBOT_PAYLOAD = 20.0
    # 팔을 세우고 손목을 아래로 (팔레타이징 기본 자세)
    # joint_1 은 픽(방위 79도)과 플레이스(방위 0도) 사이를 보게 둔다.
    READY_JOINTS_DEG = [40.0, -60.0, 100.0, 0.0, 60.0, 0.0]
    # H2017 은 링크 합계 약 75kg 이라 M0609 용 게인(1e8)으로는 솔버가
    # 발산해 클램프되고 팔이 목표를 못 따라간다. 물리적으로 말이 되는 값으로.
    DRIVE_STIFFNESS = 2.0e6
    DRIVE_DAMPING = 2.0e5

    # 흡착 그리퍼도 큰 박스에 맞춰 키운다 (5호 480x360 윗면)
    SUCTION_CUP_RADIUS = 0.070
    SUCTION_MOUNT_Z = 0.12
    # 사양서: Ø50 x 4패드, 실용 흡착력 16 kgf = 157 N.
    # 유효 가반 18kg 보다 낮으므로 흡착력이 실질 상한이다.
    # 5호 9kg = 88 N -> 여유 1.8배.
    GRIP_COAXIAL_FORCE_LIMIT = float(_os.environ.get("PL_COAXIAL", 157.0))
    GRIP_SHEAR_FORCE_LIMIT = float(_os.environ.get("PL_SHEAR", 80.0))
    # 부착 조인트 드라이브. Isaac 샘플 값(transZ 5000 / rot 100~10000, 감쇠 0)은
    # 작은 물체 기준이라 5~9kg 박스가 매달리면 흔들리다 떨어진다.
    # 강성을 올리고 회전축에도 감쇠를 준다.
    # rotX/rotY 가 무르면 박스가 기울어 모서리부터 닿는다. 480mm 박스가
    # 1도만 기울어도 모서리가 4mm 내려가는데, 놓는 여유가 그 정도다.
    SUCTION_DRIVE = {
        "transZ": (5.0e5, 5.0e4),
        "rotX":   (5.0e5, 5.0e4),
        "rotY":   (5.0e5, 5.0e4),
        "rotZ":   (5.0e5, 5.0e4),
    }
else:
    USD_PATH = M0609_USD
    URDF_PATH = M0609_URDF
    DESCRIPTION_PATH = M0609_DESC
    ROBOT_NAME = "M0609"
    ROBOT_REACH = 0.900
    ROBOT_PAYLOAD = 6.0
    READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 흡착판 치수가 프리셋에서 바뀌었을 수 있으므로 TCP 오프셋을 다시 계산한다
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_MOUNT_Z + SUCTION_CUP_THICKNESS / 2.0])

# 박스 혼입 비율. 지정이 없으면 균등.
if not globals().get("BOX_RATIO"):
    BOX_RATIO = {k: 1.0 / len(BOX_NAMES) for k in BOX_NAMES} if BOX_NAMES else {}
if not globals().get("BOX_GRADE"):
    BOX_GRADE = {k: "" for k in BOX_NAMES}

SPAWN_JITTER_XY = 0.03
SPAWN_YAW_RANGE_DEG = (-25.0, 25.0)

PALLET_PATH = "/World/Pallet"
PALLET_ORIGIN_XY = PALLET_CENTER_XY - PALLET_SIZE / 2.0
PALLET_USD = globals().get("PALLET_USD", None)
PALLET_ASSET = globals().get("PALLET_ASSET", "box")

BOX_ROOT = "/World/Boxes"
RANDOM_SEED = 42

# 1톤 트럭 적재함 (현대 포터2 장축 카고) 길이 x 폭.
# 폭은 1600 이 아니라 1630 이다 — EUR1 두 장(802 x 2 = 1604)이 들어가려면
# 이 30mm 가 필요하다.
TRUCK_BED = np.array([2.80, 1.60])      # 사양: 2800 x 1600 x 1600
TRUCK_INNER_H = 1.60
TRUCK_FLOOR_H = globals().get("TRUCK_FLOOR_H", 0.900)

# 적재 장수는 공칭 1200x800 이 아니라 실치수로 따진다. Isaac pallet.usd 는
# EUR1 1213 x 802 라 802 x 2 = 1604 > 1600 이 되어 폭 방향 2열이 안 들어간다.
# 공칭값으로 계산하면 2x2 = 4 장이 나오지만 실제로는 안 실린다.
# 800 을 길이 방향으로 세워 3장 (2400 <= 2800, 1200 <= 1600, 여유 400씩).
PALLET_FOOTPRINT = np.array([1.213, 0.802])
PALLET_LAYOUT = (
    max(int(TRUCK_BED[0] // PALLET_FOOTPRINT[0]) * int(TRUCK_BED[1] // PALLET_FOOTPRINT[1]),
        int(TRUCK_BED[0] // PALLET_FOOTPRINT[1]) * int(TRUCK_BED[1] // PALLET_FOOTPRINT[0])),
    1,
)


# ─────────────────────────────────────────────────────────────
# 패킹 알고리즘
# ─────────────────────────────────────────────────────────────
# 하이트맵 격자. 발자국은 이 격자 배수로 올림되므로, 같은 규격 박스끼리는
# 항상 같은 셀 수를 차지해 줄이 자동으로 맞는다 (오와열의 핵심).
# 우체국 규격은 전부 10mm 단위라 postal 에서는 반올림 오차가 0 이다.
PACK_CELL = 0.010 if PRESET in ("postal", "h2017") else 0.005
PACK_YAWS_DEG = (0.0, 90.0)                      # 허용 회전
PACK_MIN_SUPPORT_RATIO = 0.85                    # 밑면이 이만큼은 받쳐져야 배치 허용
PACK_SUPPORT_TOL = 0.004                         # 이 오차 안이면 "받쳐진 것"으로 간주
PACK_FLATNESS_WEIGHT = 1.0                       # 평탄도 penalty 가중치
PACK_WALL_MARGIN = 0.000 if PRESET in ("postal", "h2017") else 0.002

# 무거운 박스를 가벼운 박스 위에 올리지 않는다.
# 실측: 활용률은 0.7%p 만 손해보고 "무거운 것 위" 사례가 0 이 된다.
PACK_ENFORCE_LOAD_ORDER = True

# 박스 사이에 남기는 여유 [m] (양쪽 합).
# 내려놓을 때 박스는 옆 박스 높이를 지나쳐 내려간다. 간격이 좁으면
# 그 순간 부딪혀 흡착이 풀린다(실측: 16mm 간격에서 전부 실패).
# 배치 오차 5mm + 여유를 감안해 양쪽 25mm 씩 준다.
PACK_BOX_CLEARANCE = 0.030 if PRESET == "h2017" else 0.0

# 하강 통로 여유 [m]. 놓을 자리 주변 이 범위 안에 자기보다 높은 적재물이
# 있으면 그 자리를 쓰지 않는다 (위에서 수직으로 내려놓지 못하므로).
PACK_DESCENT_HALO = 0.040 if PRESET == "h2017" else 0.0


# ─────────────────────────────────────────────────────────────
# 모션
# ─────────────────────────────────────────────────────────────
APPROACH_CLEARANCE = 0.10                        # 픽/플레이스 대상 위 접근 높이
# 이동 높이는 고정하지 않는다. 매 사이클마다
#   max(집는 박스 위, 놓을 자리 위, 현재 적재 최고점) + TRANSIT_CLEARANCE
# 로 계산해 불필요하게 높이 들지 않는다 (리치 여유 확보).
TRANSIT_CLEARANCE = 0.10
# 상한에 운반 중인 박스 높이를 포함해야 한다. 박스는 윗면이 흡착되므로
# TCP 높이 - 박스높이 = 박스 밑면이고, 이게 적재 최고점보다 높아야 한다.
TRANSIT_Z_MAX = (PALLET_DECK_Z + PALLET_MAX_STACK_H
                 + BOX_MAX_H + TRANSIT_CLEARANCE)
# 집을 때 박스 윗면 위 이만큼에서 멈춘다.
# 끝까지 눌러 내려가면 흡착 조인트가 압축되어 힘이 수천 N 으로 치솟고
# 실제 흡착 한계(157 N)를 넘겨 오히려 떨어진다.
# 흡착은 GRIP_MAX_DISTANCE(20mm) 안에서 알아서 붙는다.
PICK_APPROACH_GAP = 0.008

# PLACE_RELEASE_GAP 은 프리셋에서 정한다. 0 으로 두면 박스를 적재면에
# 짓눌러 물리가 폭발한다 (드라이브가 최대 힘으로 계속 아래로 밀기 때문).

TCP_SPEED = 0.005                                # step 당 TCP 이동 거리 [m]
MIN_STEPS = 40
MAX_STEPS = 700
GRIPPER_WAIT = 90                                # 흡착/해제 대기 step

# 흡착판은 항상 아래를 향함 (roll 180)
APPROACH_ROLL_DEG = 180.0
APPROACH_PITCH_DEG = 0.0


# ─────────────────────────────────────────────────────────────
# 카메라 (PERCEPTION_MODE = "camera" 일 때)
# ─────────────────────────────────────────────────────────────
CAM_PATH = "/World/ScanCamera"
CAM_POS = np.array([PICK_XY[0], PICK_XY[1], 0.75])           # 픽 존 바로 위 하향
CAM_RESOLUTION = (640, 480)
CAM_FOCAL_LENGTH = 24.0
CAM_HORIZONTAL_APERTURE = 20.955

# 포인트클라우드에서 컨베이어 상판을 잘라내는 높이 마진
CAM_PLANE_TOL = 0.012

# 스캔 ROI — 컨베이어 전체가 아니라 "스캔 창" 하나만 본다.
# 컨베이어는 로봇 베이스까지 뻗어 있어서, 발자국 전체를 ROI 로 잡으면
# 로봇 몸통이 박스보다 높은 점군이 되어 그걸 박스로 인식한다.
# 높이 천장도 같은 이유로 필요하다 (시야를 가로지르는 팔 배제).
CAM_ROI_HALF = SPAWN_JITTER_XY + BOX_MAX_DIM / 2.0 + 0.03   # 픽 존 기준 반폭
CAM_MAX_BOX_H = BOX_MAX_H + 0.03                 # 컨베이어 상판 기준 최대 박스 높이

# 피팅한 사각형의 채움률이 이보다 낮으면 인식을 의심한다.
# 정상 단일 박스는 90~100%, 두 물체가 합쳐지면 50% 근처로 떨어진다.
CAM_MIN_FILL = 0.80


# ─────────────────────────────────────────────────────────────
# 로깅
# ─────────────────────────────────────────────────────────────
LOG_INTERVAL = 90


# ─────────────────────────────────────────────────────────────
# 환경변수 오버라이드 — config.py 를 고치지 않고 빠르게 실험할 때
#   PL_HEADLESS=1 PL_N_BOXES=3 ~/isaacsim/python.sh run_line.py
# ─────────────────────────────────────────────────────────────
import os as _os


def _env_bool(name, default):
    raw = _os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


HEADLESS = _env_bool("PL_HEADLESS", HEADLESS)
# 헤드리스에는 Play 버튼이 없으므로 자동 시작한다
AUTO_PLAY = _env_bool("PL_AUTOPLAY", HEADLESS)
N_BOXES = int(_os.environ.get("PL_N_BOXES", N_BOXES))
PERCEPTION_MODE = _os.environ.get("PL_PERCEPTION", PERCEPTION_MODE)
PERCEPTION_COMPARE = _env_bool("PL_COMPARE", PERCEPTION_COMPARE)
