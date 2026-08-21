#!/usr/bin/env python3
"""test1.usd 의 피더 컨베이어 위에 우체국 규격 3/4/5호 박스를 랜덤 스폰한다.

Isaac Sim 을 안 띄우고 Sdf 레이어만 직접 고친다 (set_conveyor_offline.py 와 동일 방식).

    ./run_spawn_boxes.sh              # 기본 10개
    PL_N=20 PL_SEED=7 ./run_spawn_boxes.sh

배치 근거
  · 피더 라인 = x=-9.4961 세로 구간(ConveyorTrack_03/_14/_15/_04, ConveyorBelt_A06 4조각).
    y=1.50 -> 9.50 구간이 +Y 로 흐르고, 그 위 A24 분기 두 개가 스퍼 라인
    (y=15.41, y=11.41)로 박스를 밀어낸다. 로봇(-2.36, 9.85)은 y=11.41 스퍼 끝에 있다.
  · 벨트 상면 z = 0.900 (평탄화 후), 벨트 폭 900mm (y 기준 ±0.45 -> 회전 후 x 기준 ±0.45).
    가장 큰 5호가 480mm 라 폭은 그대로 두고 박스만 올린다.
  · 박스가 벨트 밖으로 걸치지 않도록 요(yaw) 회전을 반영한 실제 발자국으로
    가로 지터 상한을 계산한다.

색상은 호수 고정 (카메라 인식 결과를 실제 규격과 대조하기 위해)
    3호 = 빨강 (1,0,0) / 4호 = 초록 (0,1,0) / 5호 = 파랑 (0,0,1)
displayColor 와 UsdPreviewSurface 머티리얼을 둘 다 넣는다. RTX 뷰포트/카메라는
머티리얼을, 뷰포트 단순 표시나 스크립트는 displayColor 를 본다.
"""

import math
import os
import random
import shutil
import time

from pxr import Gf, Sdf

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/project_1/test1.usd"

N_BOXES = int(os.environ.get("PL_N", 10))
SEED = int(os.environ.get("PL_SEED", 42))

BOX_ROOT = Sdf.Path("/World/Boxes")
LOOKS_ROOT = Sdf.Path("/World/Looks")

# config.py h2017 프리셋과 동일한 실치수 [m] / 질량 [kg] / 혼입 비율
BOX_SPECS = {
    "3호": (0.340, 0.250, 0.210),
    "4호": (0.410, 0.310, 0.280),
    "5호": (0.480, 0.360, 0.340),
}
BOX_MASS = {"3호": 3.0, "4호": 5.0, "5호": 9.0}
BOX_RATIO = {"3호": 0.50, "4호": 0.30, "5호": 0.20}
BOX_COLOR = {"3호": (1.0, 0.0, 0.0), "4호": (0.0, 1.0, 0.0), "5호": (0.0, 0.0, 1.0)}
BOX_SLUG = {"3호": "No3", "4호": "No4", "5호": "No5"}

# ── 피더 라인 (ConveyorTrack_03/_14/_15/_04) ─────────────────
LINE_X = -9.4961          # 벨트 중심선
BELT_TOP_Z = 0.900        # 평탄화 후 라인 전 구간 상면 (= 트럭 바닥고)
BELT_HALF_W = 0.45        # 폭 900mm
EDGE_MARGIN = 0.03        # 가장자리 여유
Y_START, Y_END = 1.90, 9.20
DROP_GAP = 0.005          # 벨트에 살짝 띄워 놓는다 (초기 관통 방지)
YAW_JITTER_DEG = 10.0


def main():
    backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
    shutil.copy2(USD_PATH, backup)
    print(f"백업: {backup}\n")

    layer = Sdf.Layer.FindOrOpen(USD_PATH)
    if layer is None:
        raise SystemExit(f"레이어를 못 열었다: {USD_PATH}")

    # 재실행해도 쌓이지 않게 이전 결과를 지운다
    for path in (BOX_ROOT, LOOKS_ROOT):
        if layer.GetPrimAtPath(path):
            del layer.GetPrimAtPath(path).nameParent.nameChildren[path.name]
            print(f"기존 {path} 제거")

    make_materials(layer)

    rng = random.Random(SEED)
    names = pick_sizes(rng, N_BOXES)

    boxes = define_prim(layer, BOX_ROOT, "Xform")
    step = (Y_END - Y_START) / max(N_BOXES - 1, 1)

    print(f"박스 {N_BOXES}개  (seed={SEED})")
    for i, name in enumerate(names):
        dims = BOX_SPECS[name]
        # 긴 축을 진행 방향(+Y)에 두거나 가로로 눕히거나, 반반
        yaw = (90.0 if rng.random() < 0.5 else 0.0) + rng.uniform(-YAW_JITTER_DEG, YAW_JITTER_DEG)
        c, s = abs(math.cos(math.radians(yaw))), abs(math.sin(math.radians(yaw)))
        half_x = (dims[0] * c + dims[1] * s) / 2.0     # 벨트를 가로지르는 반폭
        jitter = max(0.0, BELT_HALF_W - EDGE_MARGIN - half_x)

        x = LINE_X + rng.uniform(-jitter, jitter)
        y = Y_START + i * step
        z = BELT_TOP_Z + dims[2] / 2.0 + DROP_GAP

        prim_path = BOX_ROOT.AppendChild(f"Box_{i:02d}_{BOX_SLUG[name]}")
        make_box(layer, prim_path, name, dims, (x, y, z), yaw)
        print(f"  {prim_path.name:16s} {name}  "
              f"{dims[0]*1000:.0f}x{dims[1]*1000:.0f}x{dims[2]*1000:.0f}mm  "
              f"{BOX_MASS[name]:.0f}kg  yaw={yaw:+6.1f}도  "
              f"pos=({x:.3f}, {y:.3f}, {z:.3f})  가로여유={jitter*1000:.0f}mm")

    layer.Save()

    tally = {k: names.count(k) for k in BOX_SPECS}
    print(f"\n구성: " + " / ".join(f"{k} {v}개" for k, v in tally.items()))
    print(f"저장 완료: {USD_PATH}")


def pick_sizes(rng, n):
    """혼입 비율대로 뽑되 세 호수가 최소 하나씩은 들어가게 한다."""
    keys = list(BOX_SPECS)
    weights = [BOX_RATIO[k] for k in keys]
    while True:
        picked = rng.choices(keys, weights=weights, k=n)
        if n < len(keys) or set(picked) == set(keys):
            rng.shuffle(picked)
            return picked


# ─────────────────────────────────────────────────────────────
# Sdf 저수준 헬퍼 — Usd.Stage 를 안 열기 때문에 스펙을 직접 만든다
# (Stage 로 열면 원격 컨베이어 에셋 18개를 전부 받아온다)
# ─────────────────────────────────────────────────────────────
def define_prim(layer, path, type_name, api_schemas=None):
    spec = Sdf.CreatePrimInLayer(layer, path)
    spec.specifier = Sdf.SpecifierDef
    spec.typeName = type_name
    if api_schemas:
        spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(api_schemas))
    return spec


def set_attr(prim_spec, name, type_name, value, uniform=False):
    # variability 는 생성자에서만 정할 수 있다 (스펙 생성 후에는 읽기 전용)
    variability = Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying
    attr = Sdf.AttributeSpec(prim_spec, name, type_name, variability)
    attr.default = value
    return attr


def make_materials(layer):
    define_prim(layer, LOOKS_ROOT, "Scope")
    for name, rgb in BOX_COLOR.items():
        mat_path = LOOKS_ROOT.AppendChild(f"Box{BOX_SLUG[name]}")
        mat = define_prim(layer, mat_path, "Material")
        out = Sdf.AttributeSpec(mat, "outputs:surface", Sdf.ValueTypeNames.Token)
        shader_path = mat_path.AppendChild("Shader")
        out.connectionPathList.explicitItems.append(shader_path.AppendProperty("outputs:surface"))

        shader = define_prim(layer, shader_path, "Shader")
        set_attr(shader, "info:id", Sdf.ValueTypeNames.Token, "UsdPreviewSurface", uniform=True)
        set_attr(shader, "inputs:diffuseColor", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(*rgb))
        set_attr(shader, "inputs:roughness", Sdf.ValueTypeNames.Float, 0.6)
        set_attr(shader, "inputs:metallic", Sdf.ValueTypeNames.Float, 0.0)
        Sdf.AttributeSpec(shader, "outputs:surface", Sdf.ValueTypeNames.Token)


def make_box(layer, path, name, dims, pos, yaw_deg):
    box = define_prim(layer, path, "Cube", [
        "PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI",
        "PhysxCollisionAPI", "MaterialBindingAPI",
    ])

    # 한 변 1m 짜리 Cube 를 실치수로 스케일한다. 콜라이더도 스케일을 따라간다.
    set_attr(box, "size", Sdf.ValueTypeNames.Double, 1.0)
    set_attr(box, "extent", Sdf.ValueTypeNames.Float3Array,
             [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])

    color = set_attr(box, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
                     [Gf.Vec3f(*BOX_COLOR[name])])
    color.SetInfo("interpolation", "constant")

    set_attr(box, "physics:rigidBodyEnabled", Sdf.ValueTypeNames.Bool, True)
    set_attr(box, "physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, False)
    set_attr(box, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, True)
    set_attr(box, "physics:mass", Sdf.ValueTypeNames.Float, BOX_MASS[name])

    half = math.radians(yaw_deg) / 2.0
    set_attr(box, "xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*pos))
    set_attr(box, "xformOp:orient", Sdf.ValueTypeNames.Quatd,
             Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half))))
    set_attr(box, "xformOp:scale", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*dims))
    set_attr(box, "xformOpOrder", Sdf.ValueTypeNames.TokenArray,
             ["xformOp:translate", "xformOp:orient", "xformOp:scale"], uniform=True)

    rel = Sdf.RelationshipSpec(box, "material:binding", False)
    rel.targetPathList.explicitItems.append(LOOKS_ROOT.AppendChild(f"Box{BOX_SLUG[name]}"))


if __name__ == "__main__":
    main()
