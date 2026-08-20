"""
2.5D 하이트맵 기반 온라인 3D 패킹 엔진

- 미래에 올 박스를 모르는 상태에서, 도착한 박스 하나의 놓을 자리를 즉시 결정한다.
- 상태는 팔레트 상판을 격자로 나눈 하이트맵 하나뿐이다.
- 후보 자리를 전부 훑어 (배치 후 높이, 평탄도)로 점수를 매기고 가장 좋은 자리를 고른다.

근거: Wang & Hauser (2019), "Stable bin packing of non-convex 3D objects with a
robot manipulator" 의 Heightmap-Minimization heuristic 을 직육면체 + yaw 0/90 로
단순화한 버전.

Isaac Sim 에 의존하지 않는다 -> `python3 packing.py` 로 단독 검증 가능.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Placement:
    """패킹 결과 한 건. 좌표는 전부 팔레트 로컬 [m]."""

    x: float          # 밑면 -x 모서리 (팔레트 로컬)
    y: float          # 밑면 -y 모서리 (팔레트 로컬)
    z: float          # 밑면 높이 (팔레트 상판 기준)
    w: float          # yaw 적용 후 x 방향 길이
    d: float          # yaw 적용 후 y 방향 길이
    h: float          # 높이
    yaw_deg: float    # 원래 박스를 얼마나 돌렸는지
    support_ratio: float
    flatness: float   # 지지면 높이 표준편차 [m] (작을수록 평평)
    mass: float = 0.0 # 박스 무게 [kg] — 하중 제약 검사에 쓴다

    @property
    def center_local(self) -> np.ndarray:
        """박스 중심 (팔레트 로컬)."""
        return np.array([self.x + self.w / 2.0,
                         self.y + self.d / 2.0,
                         self.z + self.h / 2.0])

    @property
    def top_center_local(self) -> np.ndarray:
        """박스 윗면 중심 (팔레트 로컬) — 흡착 지점."""
        return np.array([self.x + self.w / 2.0,
                         self.y + self.d / 2.0,
                         self.z + self.h])


class HeightmapPacker:
    """팔레트 하나에 대한 온라인 패킹 상태."""

    def __init__(
        self,
        size_xy,
        max_height: float,
        cell: float = 0.005,
        yaws_deg=(0.0, 90.0),
        min_support_ratio: float = 0.85,
        support_tol: float = 0.004,
        flatness_weight: float = 1.0,
        wall_margin: float = 0.002,
        enforce_load_order: bool = True,
        descent_halo: float = 0.0,
    ):
        self.size_xy = np.asarray(size_xy, dtype=float)
        self.max_height = float(max_height)
        self.cell = float(cell)
        self.yaws_deg = tuple(yaws_deg)
        self.min_support_ratio = float(min_support_ratio)
        self.support_tol = float(support_tol)
        self.flatness_weight = float(flatness_weight)
        self.wall_margin = float(wall_margin)
        # 무거운 박스를 가벼운 박스 위에 올리지 않는다.
        # 물류 기본 원칙이고, 없으면 9kg 5호가 3kg 3호를 깔고 앉는다.
        self.enforce_load_order = bool(enforce_load_order)
        # 하강 통로. 로봇은 박스를 위에서 수직으로 내려놓으므로, 놓을 자리
        # 주변에 자기보다 높은 이웃이 있으면 그 옆을 스쳐 내려가야 하고
        # 실제로 부딪힌다. 기하학적으로 들어간다고 넣을 수 있는 게 아니다.
        self.descent_halo = float(descent_halo)

        usable = self.size_xy - 2.0 * self.wall_margin
        self.nx = max(1, int(round(usable[0] / self.cell)))
        self.ny = max(1, int(round(usable[1] / self.cell)))

        self.heightmap = np.zeros((self.nx, self.ny), dtype=np.float64)
        # 각 칸의 "맨 위 박스 무게". 바닥은 무한(팔레트가 다 받는다).
        self.massmap = np.full((self.nx, self.ny), np.inf, dtype=np.float64)
        self.placements: list[Placement] = []
        self.rejected: list[tuple[float, float, float]] = []

    # ── 상태 ────────────────────────────────────────────────
    def reset(self) -> None:
        self.heightmap[:] = 0.0
        self.massmap[:] = np.inf
        self.placements.clear()
        self.rejected.clear()

    @property
    def current_height(self) -> float:
        return float(self.heightmap.max())

    def volume_utilization(self) -> float:
        """팔레트 전체 부피 대비 적재된 박스 부피 비율."""
        bin_volume = float(self.size_xy[0] * self.size_xy[1] * self.max_height)
        if bin_volume <= 0.0:
            return 0.0
        packed = sum(p.w * p.d * p.h for p in self.placements)
        return packed / bin_volume

    def occupied_utilization(self) -> float:
        """실제로 쌓인 높이까지만 놓고 본 활용률 (빈 공간 비율의 역)."""
        h = self.current_height
        if h <= 0.0:
            return 0.0
        envelope = float(self.size_xy[0] * self.size_xy[1]) * h
        packed = sum(p.w * p.d * p.h for p in self.placements)
        return packed / envelope

    # ── 좌표 변환 ───────────────────────────────────────────
    def _cells(self, length: float) -> int:
        """길이를 덮는 데 필요한 셀 수 (올림 — 겹침을 과소평가하지 않도록)."""
        return max(1, int(np.ceil(length / self.cell - 1e-9)))

    def local_to_world(self, local_xyz, pallet_origin_xy, deck_z: float) -> np.ndarray:
        """팔레트 로컬 -> 월드. wall_margin 만큼 안쪽으로 들어가 있음에 주의."""
        local_xyz = np.asarray(local_xyz, dtype=float)
        return np.array([
            pallet_origin_xy[0] + self.wall_margin + local_xyz[0],
            pallet_origin_xy[1] + self.wall_margin + local_xyz[1],
            deck_z + local_xyz[2],
        ])

    # ── 핵심: 자리 찾기 ─────────────────────────────────────
    def _scan(self, fw: int, fd: int, box_h: float, box_mass: float = 0.0):
        """
        fw x fd 셀 크기의 발자국을 모든 위치에 놓아보고 후보별 지표를 계산한다.

        sliding_window_view 로 한 번에 계산 -> 격자를 촘촘히 해도 실시간으로 돈다.

        Returns: (base_h, flatness, support, valid) 각각 shape (nx-fw+1, ny-fd+1)
                 또는 발자국이 팔레트보다 크면 None
        """
        if fw > self.nx or fd > self.ny:
            return None

        windows = np.lib.stride_tricks.sliding_window_view(
            self.heightmap, (fw, fd)
        )  # (nx-fw+1, ny-fd+1, fw, fd)

        base_h = windows.max(axis=(2, 3))
        flatness = windows.std(axis=(2, 3))

        # 지지율: 발자국 중 "가장 높은 면"과 같은 높이에 있는 셀의 비율.
        # 이게 낮으면 박스가 허공에 걸쳐진다.
        supported = windows >= (base_h[..., None, None] - self.support_tol)
        support = supported.mean(axis=(2, 3))

        valid = (base_h + box_h <= self.max_height + 1e-9) & (
            support >= self.min_support_ratio - 1e-9
        )

        halo_cells = int(np.ceil(self.descent_halo / self.cell)) if self.descent_halo else 0
        if halo_cells > 0:
            # 발자국 주변 halo 만큼 넓힌 영역의 최고 높이를 본다.
            # 바깥은 -inf 로 채워 팔레트 경계가 통로를 막지 않게 한다.
            padded = np.pad(self.heightmap, halo_cells,
                            mode="constant", constant_values=-np.inf)
            halo_win = np.lib.stride_tricks.sliding_window_view(
                padded, (fw + 2 * halo_cells, fd + 2 * halo_cells)
            )
            halo_max = halo_win.max(axis=(2, 3))
            # 이웃이 내가 놓인 뒤의 윗면보다 높으면 내려갈 통로가 없다
            valid = valid & (halo_max <= base_h + box_h + 1e-9)

        if self.enforce_load_order and box_mass > 0.0:
            # 발자국이 덮는 칸들 중 가장 가벼운 지지 박스가 이 박스보다
            # 가벼우면 배치 불가. 바닥(inf)은 항상 통과한다.
            mass_win = np.lib.stride_tricks.sliding_window_view(
                self.massmap, (fw, fd)
            )
            min_support_mass = mass_win.min(axis=(2, 3))
            valid = valid & (min_support_mass >= box_mass - 1e-9)

        return base_h, flatness, support, valid

    def find_placement(self, box_size, box_mass: float = 0.0) -> Placement | None:
        """
        박스 하나(w, d, h)의 최적 배치를 찾는다. 실패하면 None.

        점수는 사전식(lexicographic):
          1) 배치 후 최고 높이가 낮을수록 좋다        <- 하이트맵 최소화
          2) 지지면이 평평할수록 좋다                 <- 안정성
          3) 팔레트 모서리에 가까울수록 좋다          <- 파편화 억제
        """
        w0, d0, h = (float(v) for v in box_size)

        best = None
        best_key = None

        for yaw in self.yaws_deg:
            if abs(yaw % 180.0) < 1e-6:
                w, d = w0, d0
            elif abs(yaw % 180.0 - 90.0) < 1e-6:
                w, d = d0, w0
            else:
                raise ValueError(f"yaw {yaw} 는 0/90 만 지원합니다")

            fw, fd = self._cells(w), self._cells(d)
            scanned = self._scan(fw, fd, h, box_mass)
            if scanned is None:
                continue

            base_h, flatness, support, valid = scanned
            if not valid.any():
                continue

            resulting_h = base_h + h

            # 유효하지 않은 후보는 경쟁에서 제외
            big = np.inf
            score_h = np.where(valid, resulting_h, big)
            score_f = np.where(valid, flatness * self.flatness_weight, big)

            # 모서리 근접도 (셀 인덱스 합) — 같은 높이/평탄도면 구석부터 채운다
            ii, jj = np.meshgrid(
                np.arange(score_h.shape[0]),
                np.arange(score_h.shape[1]),
                indexing="ij",
            )
            score_c = np.where(valid, ii + jj, big)

            # 사전식 최소값을 하나의 정렬 키로 압축
            order = np.lexsort((score_c.ravel(), score_f.ravel(), score_h.ravel()))
            idx = order[0]
            i, j = np.unravel_index(idx, score_h.shape)

            if not valid[i, j]:
                continue

            key = (float(score_h[i, j]), float(score_f[i, j]), float(score_c[i, j]))
            if best_key is None or key < best_key:
                best_key = key
                best = Placement(
                    x=i * self.cell,
                    y=j * self.cell,
                    z=float(base_h[i, j]),
                    w=w,
                    d=d,
                    h=h,
                    yaw_deg=yaw,
                    support_ratio=float(support[i, j]),
                    flatness=float(flatness[i, j]),
                    mass=float(box_mass),
                )

        return best

    def commit(self, placement: Placement) -> None:
        """배치를 확정하고 하이트맵을 갱신한다."""
        i0 = int(round(placement.x / self.cell))
        j0 = int(round(placement.y / self.cell))
        i1 = i0 + self._cells(placement.w)
        j1 = j0 + self._cells(placement.d)

        self.heightmap[i0:i1, j0:j1] = placement.z + placement.h
        self.massmap[i0:i1, j0:j1] = placement.mass if placement.mass > 0 else np.inf
        self.placements.append(placement)

    def undo_last(self) -> Placement | None:
        """
        마지막 배치를 취소하고 하이트맵을 다시 계산한다.

        로봇이 박스를 놓치거나 엉뚱한 곳에 놓으면 하이트맵이 실제와 달라진다.
        그 상태로 다음 박스를 계산하면 있지도 않은 지지면 위에 쌓게 되므로,
        실패한 배치는 반드시 되돌려야 한다.
        """
        if not self.placements:
            return None

        removed = self.placements.pop()
        self.heightmap[:] = 0.0
        self.massmap[:] = np.inf
        for p in self.placements:
            i0 = int(round(p.x / self.cell))
            j0 = int(round(p.y / self.cell))
            i1, j1 = i0 + self._cells(p.w), j0 + self._cells(p.d)
            self.heightmap[i0:i1, j0:j1] = p.z + p.h
            self.massmap[i0:i1, j0:j1] = p.mass if p.mass > 0 else np.inf
        return removed

    def place(self, box_size, box_mass: float = 0.0) -> Placement | None:
        """find + commit 을 한 번에. 실패하면 None 을 반환하고 기록만 남긴다."""
        p = self.find_placement(box_size, box_mass)
        if p is None:
            self.rejected.append(tuple(float(v) for v in box_size))
            return None
        self.commit(p)
        return p


# ─────────────────────────────────────────────────────────────
# 단독 검증
# ─────────────────────────────────────────────────────────────
def _selftest():
    import time

    import config as C

    rng = np.random.default_rng(C.RANDOM_SEED)

    def make_packer():
        return HeightmapPacker(
            size_xy=C.PALLET_SIZE,
            max_height=C.PALLET_MAX_STACK_H,
            cell=C.PACK_CELL,
            yaws_deg=C.PACK_YAWS_DEG,
            min_support_ratio=C.PACK_MIN_SUPPORT_RATIO,
            support_tol=C.PACK_SUPPORT_TOL,
            flatness_weight=C.PACK_FLATNESS_WEIGHT,
            wall_margin=C.PACK_WALL_MARGIN,
        )

    probe = make_packer()
    print("=" * 66)
    print(" 하이트맵 온라인 패킹 — 단독 검증")
    print("=" * 66)
    print(f" 팔레트     {C.PALLET_SIZE[0]*1000:.0f} x {C.PALLET_SIZE[1]*1000:.0f} mm"
          f"  최대높이 {C.PALLET_MAX_STACK_H*1000:.0f} mm")
    print(f" 격자       {probe.nx} x {probe.ny}  (cell {C.PACK_CELL*1000:.0f} mm)")
    if C.BOX_MODE == "spec":
        for name, d in C.BOX_SPECS.items():
            print(f" 박스 {name}   {d[0]*1000:5.1f} x {d[1]*1000:5.1f} x {d[2]*1000:5.1f} mm"
                  f"   ({C.BOX_MASS[name]} kg)")
    else:
        print(f" 박스       {C.BOX_MIN*1000:.0f} ~ {C.BOX_MAX*1000:.0f} mm 랜덤")

    def sample_box():
        if C.BOX_MODE == "spec":
            name = C.BOX_NAMES[int(rng.integers(len(C.BOX_NAMES)))]
            return C.BOX_SPECS[name], C.BOX_MASS[name]
        d = rng.uniform(C.BOX_MIN, C.BOX_MAX, size=3)
        return d, float(np.prod(d) * C.BOX_DENSITY)
    print(f" 최소 지지율 {C.PACK_MIN_SUPPORT_RATIO:.2f}")
    print()

    n_trials = 30
    n_boxes = 40

    utils, occ_utils, placed_rates, heights, timings = [], [], [], [], []

    for _ in range(n_trials):
        packer = make_packer()
        n_placed = 0
        for _ in range(n_boxes):
            box, mass = sample_box()
            t0 = time.perf_counter()
            result = packer.place(box, mass)
            timings.append((time.perf_counter() - t0) * 1000.0)
            if result is not None:
                n_placed += 1

        utils.append(packer.volume_utilization())
        occ_utils.append(packer.occupied_utilization())
        placed_rates.append(n_placed / n_boxes)
        heights.append(packer.current_height)

    print(f" {n_trials}회 x {n_boxes}박스")
    print(f"   부피 활용률(팔레트 전체)  {np.mean(utils)*100:5.1f} %"
          f"   (±{np.std(utils)*100:.1f}%p)")
    print(f"   부피 활용률(쌓인 높이까지) {np.mean(occ_utils)*100:5.1f} %"
          f"   (±{np.std(occ_utils)*100:.1f}%p)")
    print(f"   배치 성공률                {np.mean(placed_rates)*100:5.1f} %")
    print(f"   최종 적재 높이             {np.mean(heights)*1000:5.1f} mm"
          f"   / 한계 {C.PALLET_MAX_STACK_H*1000:.0f} mm")
    print()
    print(f"   1개 배치 결정 시간  평균 {np.mean(timings):.2f} ms"
          f"  최대 {np.max(timings):.2f} ms")
    print(f"   -> 실시간 여유 {'OK' if np.max(timings) < 50 else '주의: 60Hz 루프에 부담'}")

    # 지지율 하한이 실제로 지켜지는지 (안정성 불변식)
    packer = make_packer()
    for _ in range(60):
        packer.place(*sample_box())

    bad = [p for p in packer.placements
           if p.support_ratio < C.PACK_MIN_SUPPORT_RATIO - 1e-6]
    over = [p for p in packer.placements
            if p.z + p.h > C.PALLET_MAX_STACK_H + 1e-6]

    print()
    print(f" 불변식 검사 ({len(packer.placements)}개 배치)")
    print(f"   지지율 위반   {len(bad)} 건  {'OK' if not bad else 'FAIL'}")
    print(f"   높이 초과     {len(over)} 건  {'OK' if not over else 'FAIL'}")

    # 겹침 검사 — 두 박스의 AABB 가 3축 모두에서 겹치면 실패
    overlaps = 0
    ps = packer.placements
    for a in range(len(ps)):
        for b in range(a + 1, len(ps)):
            p, q = ps[a], ps[b]
            eps = 1e-6
            if (min(p.x + p.w, q.x + q.w) - max(p.x, q.x) > eps
                    and min(p.y + p.d, q.y + q.d) - max(p.y, q.y) > eps
                    and min(p.z + p.h, q.z + q.h) - max(p.z, q.z) > eps):
                overlaps += 1
    print(f"   박스 겹침     {overlaps} 건  {'OK' if not overlaps else 'FAIL'}")
    print()


if __name__ == "__main__":
    _selftest()
