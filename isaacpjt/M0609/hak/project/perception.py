"""
3D 인식 — 하나의 인터페이스, 두 개의 구현

    BoxPerception.observe() -> BoxObservation | None

  GroundTruthPerception : 박스 USD prim 에서 치수/자세를 직접 읽는다.
                          파이프라인 전체를 먼저 돌리기 위한 기준 구현.
  CameraPerception      : 하향 카메라의 depth -> 포인트클라우드 -> 상판 제거 ->
                          최소면적 사각형 피팅으로 치수/자세를 추정한다.

두 구현은 완전히 같은 값을 내놓으므로 config.PERCEPTION_MODE 로 갈아끼울 수 있고,
같은 씬에서 나란히 돌려 오차를 바로 측정할 수 있다 (compare() 참고).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pxr import Usd, UsdGeom

import config as C


@dataclass
class BoxObservation:
    """인식된 박스 하나."""

    size: np.ndarray        # (a, b, h) — 박스 자체 좌표계 기준 [m]
    yaw_deg: float          # 박스 로컬 x축이 월드 x축에서 돌아간 각 [deg]
    top_center: np.ndarray  # 윗면 중심 월드 좌표 [m] — 흡착 지점
    source: str             # "gt" | "camera"
    fill: float = 1.0       # 피팅한 사각형이 점으로 얼마나 채워졌나 (0~1)

    def __str__(self):
        a, b, h = self.size
        return (f"{a*1000:5.1f} x {b*1000:5.1f} x {h*1000:5.1f} mm"
                f"  yaw {self.yaw_deg:+6.1f}°"
                f"  top [{self.top_center[0]:+.3f} {self.top_center[1]:+.3f}"
                f" {self.top_center[2]:+.3f}]"
                + (f"  fill {self.fill*100:.0f}%" if self.source == "camera" else ""))


# ─────────────────────────────────────────────────────────────
# 공통 유틸
# ─────────────────────────────────────────────────────────────
def _yaw_from_matrix(m: np.ndarray) -> float:
    """회전행렬에서 z축 회전각을 뽑는다."""
    return float(np.degrees(np.arctan2(m[1, 0], m[0, 0])))


def _normalize_yaw(deg: float) -> float:
    """직육면체는 90° 대칭이므로 [-45, 45) 로 접는다."""
    deg = (deg + 45.0) % 90.0 - 45.0
    return float(deg)


def min_area_rect(points_xy: np.ndarray, angle_step_deg: float = 0.5):
    """
    XY 점군을 감싸는 최소 면적 직사각형.

    회전 캘리퍼스 대신 0~90° 를 훑는 방식 — scipy 없이 돌고, 0.5° 해상도면
    치수 오차가 박스 크기의 1% 미만이라 실용상 충분하다.

    Returns: (center_xy, (w, d), yaw_deg)
    """
    best = None
    for deg in np.arange(0.0, 90.0, angle_step_deg):
        rad = np.radians(deg)
        c, s = np.cos(rad), np.sin(rad)
        rot = np.array([[c, s], [-s, c]])          # 점군을 -deg 만큼 돌림
        local = points_xy @ rot.T
        lo, hi = local.min(axis=0), local.max(axis=0)
        extent = hi - lo
        area = float(extent[0] * extent[1])
        if best is None or area < best[0]:
            center_local = (lo + hi) / 2.0
            center_world = center_local @ rot            # rot.T 의 역 = rot
            best = (area, center_world, extent.copy(), float(deg))

    _, center, extent, deg = best
    return center, extent, _normalize_yaw(deg)


# ─────────────────────────────────────────────────────────────
# 기준 구현 — Ground Truth
# ─────────────────────────────────────────────────────────────
class GroundTruthPerception:
    """USD prim 의 로컬 extent + 스케일 + 회전에서 치수/자세를 그대로 읽는다."""

    source = "gt"

    def __init__(self, stage):
        self._stage = stage

    def observe(self, box_prim_path: str) -> BoxObservation | None:
        prim = self._stage.GetPrimAtPath(box_prim_path)
        if not prim.IsValid():
            return None

        xform = UsdGeom.Xformable(prim)
        mat = np.array(
            xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        ).T  # pxr 은 row-vector 규약이라 전치해서 표준 4x4 로 맞춘다

        rot = mat[:3, :3]
        pos = mat[:3, 3]

        # 스케일은 회전행렬 각 열의 노름
        scale = np.linalg.norm(rot, axis=0)
        rot_pure = rot / scale

        boundable = UsdGeom.Boundable(prim)
        extent_attr = boundable.GetExtentAttr().Get()
        if extent_attr is not None:
            lo = np.array(extent_attr[0], dtype=float)
            hi = np.array(extent_attr[1], dtype=float)
            local_size = hi - lo
        else:
            local_size = np.ones(3)

        size = local_size * scale
        yaw = _normalize_yaw(_yaw_from_matrix(rot_pure))

        top_center = pos + rot_pure @ np.array([0.0, 0.0, size[2] / 2.0])

        return BoxObservation(
            size=size,
            yaw_deg=yaw,
            top_center=top_center,
            source=self.source,
        )


# ─────────────────────────────────────────────────────────────
# 카메라 구현
# ─────────────────────────────────────────────────────────────
class CameraPerception:
    """
    하향 카메라 depth -> 월드 포인트클라우드 -> 컨베이어 상판 제거 ->
    남은 점군에 최소면적 사각형 피팅.

    컨베이어 위에는 한 번에 박스 하나만 올라오므로 별도 군집화 없이
    픽 존 반경 안의 점만 쓴다. (2차에서 다중 박스로 갈 때 군집화 추가)
    """

    source = "camera"

    def __init__(self, camera):
        self._camera = camera

    def _pointcloud_world(self) -> np.ndarray | None:
        depth = self._camera.get_depth()
        if depth is None:
            return None
        depth = np.asarray(depth, dtype=np.float64)
        if depth.ndim != 2 or not np.isfinite(depth).any():
            return None

        K = np.asarray(self._camera.get_intrinsics_matrix(), dtype=np.float64)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        h, w = depth.shape
        uu, vv = np.meshgrid(np.arange(w), np.arange(h))

        valid = np.isfinite(depth) & (depth > 1e-3) & (depth < 1e3)
        if not valid.any():
            return None

        z = depth[valid]
        x = (uu[valid] - cx) / fx * z
        y = (vv[valid] - cy) / fy * z

        # Isaac 카메라 광학 규약: +X 오른쪽, +Y 아래, +Z 전방
        pts_cam = np.stack([x, y, z], axis=1)

        cam_pos, cam_quat = self._camera.get_world_pose(camera_axes="usd")
        R = _quat_to_matrix(np.asarray(cam_quat, dtype=float))

        # USD 카메라는 -Z 를 바라보고 +Y 가 위 -> 광학축 규약에서 변환
        optical_to_usd = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

        return pts_cam @ optical_to_usd.T @ R.T + np.asarray(cam_pos, dtype=float)

    def observe(self, box_prim_path: str | None = None) -> BoxObservation | None:
        """box_prim_path 는 무시한다 — 카메라는 씬을 그대로 본다."""
        pts = self._pointcloud_world()
        if pts is None or len(pts) == 0:
            return None

        # ── ROI ────────────────────────────────────────────
        # 픽 존 위 스캔 창 안에서, 상판 위 ~ 최대 박스 높이까지만 본다.
        # 높이 천장이 핵심이다. 이게 없으면 시야를 가로지르는 로봇 팔이
        # 가장 높은 점군이 되어 팔을 박스로 인식한다.
        floor_z = C.CONVEYOR_TOP_Z + C.CAM_PLANE_TOL
        ceil_z = C.CONVEYOR_TOP_Z + C.CAM_MAX_BOX_H

        d_xy = np.abs(pts[:, :2] - C.PICK_XY)

        mask = (
            (pts[:, 2] > floor_z)
            & (pts[:, 2] < ceil_z)
            & (d_xy[:, 0] < C.CAM_ROI_HALF)
            & (d_xy[:, 1] < C.CAM_ROI_HALF)
        )
        box_pts = pts[mask]
        if len(box_pts) < 50:
            return None

        # ── 윗면 추출 -> 군집화 (순서가 중요하다) ───────────
        # 옆면 점을 먼저 버려야 한다. ROI 전체를 군집화하면 나란히 놓인
        # 두 물체가 옆면을 통해 하나의 연결 성분으로 이어져 버리고,
        # 결과적으로 둘을 감싸는 거대한 사각형이 피팅된다.
        top_z = float(np.percentile(box_pts[:, 2], 98))
        top_pts = box_pts[box_pts[:, 2] > top_z - 0.010]
        if len(top_pts) < 30:
            return None

        top_pts = _largest_cluster_near(top_pts, C.PICK_XY)
        if top_pts is None or len(top_pts) < 30:
            return None

        # 군집을 고른 뒤 윗면 높이를 다시 잰다 (다른 물체가 더 높았을 수 있음)
        top_z = float(np.percentile(top_pts[:, 2], 98))

        center_xy, extent, yaw = min_area_rect(top_pts[:, :2])
        fill = rect_fill_ratio(top_pts[:, :2], center_xy, extent, yaw)

        height = top_z - C.CONVEYOR_TOP_Z

        return BoxObservation(
            size=np.array([extent[0], extent[1], height]),
            yaw_deg=yaw,
            top_center=np.array([center_xy[0], center_xy[1], top_z]),
            source=self.source,
            fill=fill,
        )


def rect_fill_ratio(points_xy: np.ndarray, center_xy, extent, yaw_deg: float,
                    cell: float = 0.006) -> float:
    """
    피팅한 사각형 안에서 점이 실제로 덮은 셀의 비율.

    단일 박스의 윗면이면 1.0 에 가깝다. 두 물체가 붙어 하나로 피팅되면
    가운데가 비어 값이 뚝 떨어지므로, 잘못된 인식을 잡아내는 게이트가 된다.
    """
    rad = np.radians(yaw_deg)
    c, s = np.cos(rad), np.sin(rad)
    rot = np.array([[c, s], [-s, c]])
    local = (points_xy - np.asarray(center_xy, dtype=float)) @ rot.T

    nx = max(1, int(np.ceil(extent[0] / cell)))
    ny = max(1, int(np.ceil(extent[1] / cell)))

    ix = np.clip(((local[:, 0] + extent[0] / 2.0) / cell).astype(int), 0, nx - 1)
    iy = np.clip(((local[:, 1] + extent[1] / 2.0) / cell).astype(int), 0, ny - 1)

    grid = np.zeros((nx, ny), dtype=bool)
    grid[ix, iy] = True
    return float(grid.mean())


def _largest_cluster_near(pts: np.ndarray, anchor_xy, cell: float = 0.004):
    """
    XY 평면에 점을 격자화하고 4-연결 성분으로 나눈 뒤,
    anchor_xy 에 가장 가까운 성분의 점만 돌려준다.

    박스는 서로 떨어져 컨베이어에 오므로 이 정도 군집화면 충분하다.
    (박스끼리 붙어서 오는 경우는 2차에서 다룬다)
    """
    from scipy import ndimage

    xy = pts[:, :2]
    lo = xy.min(axis=0)
    idx = np.floor((xy - lo) / cell).astype(int)
    nx, ny = idx[:, 0].max() + 1, idx[:, 1].max() + 1
    if nx * ny > 4_000_000:
        return pts

    grid = np.zeros((nx, ny), dtype=bool)
    grid[idx[:, 0], idx[:, 1]] = True

    labels, n = ndimage.label(grid)
    if n <= 1:
        return pts

    point_labels = labels[idx[:, 0], idx[:, 1]]

    best, best_key = None, None
    for label in range(1, n + 1):
        sel = point_labels == label
        count = int(sel.sum())
        if count < 30:
            continue
        center = xy[sel].mean(axis=0)
        dist = float(np.linalg.norm(center - np.asarray(anchor_xy, dtype=float)))
        key = (dist, -count)
        if best_key is None or key < best_key:
            best_key, best = key, sel

    return pts[best] if best is not None else pts


def _quat_to_matrix(q) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ─────────────────────────────────────────────────────────────
# 팩토리 / 비교
# ─────────────────────────────────────────────────────────────
def build(mode: str, stage=None, camera=None):
    if mode == "gt":
        if stage is None:
            raise ValueError("gt 모드에는 stage 가 필요합니다")
        return GroundTruthPerception(stage)
    if mode == "camera":
        if camera is None:
            raise ValueError("camera 모드에는 camera 가 필요합니다")
        return CameraPerception(camera)
    raise ValueError(f"알 수 없는 PERCEPTION_MODE: {mode!r} (gt | camera)")


def compare(gt: BoxObservation | None, cam: BoxObservation | None) -> str:
    """두 인식 결과의 오차를 한 줄로. 카메라 구현 튜닝할 때 씁니다."""
    if gt is None or cam is None:
        return "   perception  비교 불가 (한쪽이 None)"

    # 박스는 90° 대칭이므로 변 길이를 정렬해서 비교
    gt_sorted = np.sort(gt.size[:2])
    cam_sorted = np.sort(cam.size[:2])
    d_wd = (cam_sorted - gt_sorted) * 1000.0
    d_h = (cam.size[2] - gt.size[2]) * 1000.0
    d_yaw = _normalize_yaw(cam.yaw_deg - gt.yaw_deg)
    d_xy = np.linalg.norm(cam.top_center[:2] - gt.top_center[:2]) * 1000.0

    return (f"   perception  치수오차 [{d_wd[0]:+.1f} {d_wd[1]:+.1f} {d_h:+.1f}] mm"
            f"  yaw {d_yaw:+.1f}°  중심 {d_xy:.1f} mm")
