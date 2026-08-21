#!/usr/bin/env python3
"""본선 컬럼 상단(ConveyorTrack_22 앞)의 벨트 끊김을 닫는다.

    ./run_fix_column_gaps.sh

증상
    y=17 근처에서 박스가 걸리거나 빠진다.

원인
    _06(A24) 북단 y=16.675 과 _22 남단 y=17.137 사이가 0.462m 비어 있다.
    3호 짧은 변이 250mm 라 그대로 빠진다.

계산
    _06 북단(16.675) 에서 커브 _07 입구(22.836) 까지 6.161m 인데
    사이에 놓인 A06 세 조각은 6.000m 다. 0.161m 가 모자라서 어디엔가
    반드시 틈이 생긴다. 그래서 세 조각을 _06 에 맞물려 올리고,
    남는 0.161m 는 커브와 그 하류 스퍼를 통째로 남쪽으로 내려 흡수한다.

    커브 출구는 A03 Anchorpoint 로 계산한다.
        exit = translate + R(+90) * (1.495, -1.596) = translate + (1.596, 1.495)
    _08(A37, 180도) 의 먼 쪽 끝이 여기에 붙어야 한다.
        _08.exit = translate + R(180) * (3.907, 0) = (translate_x - 3.907, translate_y)
    지금은 커브 출구 y=24.331, _08 이 24.42 라 89mm 어긋나 있다. 이것도 같이 맞는다.

건드리지 않는 것
    남쪽 구간(_03/_14/_05/_18/_19/_15/_06)은 틈이 없다. 겹침만 있는데
    (_19 와 _15 가 1.18m 겹친다) 높이가 같아 주행에는 지장이 없다.
    _05/_06 의 분기 앵커도 스퍼(_12, _10)와 3~20mm 안에서 맞아 있어 그대로 둔다.
"""

import shutil
import time

from pxr import Gf, Sdf

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

PIECE_LEN = 2.0            # ConveyorBelt_A06 길이
A03_EXIT_DY = 1.495        # 커브 출구의 y 오프셋 (+90도 회전 기준)


def main():
    backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
    shutil.copy2(USD_PATH, backup)
    print(f"백업: {backup}\n")

    layer = Sdf.Layer.FindOrOpen(USD_PATH)
    if layer is None:
        raise SystemExit(f"레이어를 못 열었다: {USD_PATH}")

    base = get_y(layer, "ConveyorTrack_06")          # A24 북단 = 원점 y
    curve_y = base + 3 * PIECE_LEN
    spur_y = curve_y + A03_EXIT_DY

    print(f"기준: ConveyorTrack_06 북단 y = {base:.3f}")
    print("이동 후 컬럼 상단 구간")
    for name, new_y in (
        ("ConveyorTrack_22", base + 1 * PIECE_LEN),
        ("ConveyorTrack_21", base + 2 * PIECE_LEN),
        ("ConveyorTrack_20", base + 3 * PIECE_LEN),
        ("ConveyorTrack_07", curve_y),               # 커브 입구
        ("ConveyorTrack_08", spur_y),                # 램프
        ("ConveyorTrack_09", spur_y),                # 롤러
    ):
        set_y(layer, name, new_y)

    layer.Save()
    print(f"\n저장 완료: {USD_PATH}")
    report(layer, base)


def path_of(name):
    return Sdf.Path(f"/World/{name}").AppendProperty("xformOp:translate")


def get_y(layer, name):
    return float(layer.GetAttributeAtPath(path_of(name)).default[1])


def set_y(layer, name, new_y):
    attr = layer.GetAttributeAtPath(path_of(name))
    if attr is None:
        print(f"  [없음] {name}")
        return
    old = attr.default
    attr.default = Gf.Vec3d(old[0], new_y, old[2])
    print(f"  {name:18s} y {old[1]:8.3f} -> {new_y:8.3f}   ({new_y - old[1]:+.3f})")


def report(layer, base):
    """이동 후 컬럼이 실제로 이어지는지 커버 구간으로 확인한다."""
    print("\n검증 — 컬럼 커버 구간 (x=-9.496, -90도 조각은 y..y-길이)")
    pieces = [
        ("ConveyorTrack_03", +1, 2.0), ("ConveyorTrack_14", +1, 2.0),
        ("ConveyorTrack_05", -1, 4.0), ("ConveyorTrack_18", -1, 2.0),
        ("ConveyorTrack_19", -1, 2.0), ("ConveyorTrack_15", -1, 2.0),
        ("ConveyorTrack_06", -1, 4.0), ("ConveyorTrack_22", -1, 2.0),
        ("ConveyorTrack_21", -1, 2.0), ("ConveyorTrack_20", -1, 2.0),
    ]
    spans = []
    for name, sign, length in pieces:
        y = get_y(layer, name)
        spans.append((name, (y, y + length) if sign > 0 else (y - length, y)))
    spans.sort(key=lambda s: s[1][0])

    prev_end, prev_name = None, None
    for name, (lo, hi) in spans:
        note = ""
        if prev_end is not None:
            d = lo - prev_end
            if d > 0.001:
                note = f"  <<< 틈 {d * 1000:.0f}mm  ({prev_name} 다음)"
            elif d < -0.001:
                note = f"  (겹침 {-d * 1000:.0f}mm)"
        print(f"   {name:18s} y {lo:7.3f} .. {hi:7.3f}{note}")
        prev_end, prev_name = max(prev_end or hi, hi), name

    curve_y = get_y(layer, "ConveyorTrack_07")
    print(f"   {'ConveyorTrack_07':18s} 커브 입구 y {curve_y:7.3f}  "
          f"(_20 북단과 차이 {(curve_y - prev_end) * 1000:+.0f}mm)")
    spur = get_y(layer, "ConveyorTrack_08")
    print(f"   {'커브 출구':18s} y {curve_y + A03_EXIT_DY:7.3f}  "
          f"스퍼 _08/_09 y {spur:7.3f}  차이 {(spur - curve_y - A03_EXIT_DY) * 1000:+.0f}mm")


if __name__ == "__main__":
    main()
