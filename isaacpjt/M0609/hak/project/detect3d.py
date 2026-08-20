"""
3D 인식 : 카메라 + Replicator bounding_box_3d 로 상자 치수 측정

Isaac Sim Script Editor:
    exec(open("/home/rokey/detect3d.py").read())

/World/Cube 가 스테이지에 있어야 합니다.
"""

import asyncio
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom

PRIM_PATH = "/World/Cube"
LABEL = "box"
RES = (640, 480)

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath(PRIM_PATH)
if not prim.IsValid():
    raise RuntimeError("프림이 없습니다: " + PRIM_PATH)


# ----------------------------------------------------------------------
# 1. 시맨틱 라벨 (버전마다 API가 달라서 순서대로 시도)
# ----------------------------------------------------------------------
def attach_label(p, name):
    try:
        from isaacsim.core.utils.semantics import add_labels
        add_labels(p, labels=[name], instance_name="class")
        return "isaacsim.core.utils.semantics.add_labels"
    except Exception:
        pass
    try:
        from omni.isaac.core.utils.semantics import add_update_semantics
        add_update_semantics(p, name)
        return "omni.isaac.core.utils.semantics.add_update_semantics"
    except Exception:
        pass
    from pxr import Semantics
    sem = Semantics.SemanticsAPI.Apply(p, "Semantics")
    sem.CreateSemanticTypeAttr().Set("class")
    sem.CreateSemanticDataAttr().Set(name)
    return "pxr.Semantics (legacy)"


how = attach_label(prim, LABEL)
print("[1] 라벨 부착 : %s  (%s)" % (LABEL, how))


# ----------------------------------------------------------------------
# 2. 정답값 (USD 바운딩박스) - 나중에 인식 결과와 비교용
# ----------------------------------------------------------------------
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
rngb = cache.ComputeWorldBound(prim).ComputeAlignedRange()
gt = np.array([rngb.GetSize()[0], rngb.GetSize()[1], rngb.GetSize()[2]])
mid = rngb.GetMidpoint()
center = np.array([mid[0], mid[1], mid[2]])
print("[2] 정답  L%.4f W%.4f H%.4f   중심 (%.3f, %.3f, %.3f)"
      % (gt[0], gt[1], gt[2], center[0], center[1], center[2]))


# ----------------------------------------------------------------------
# 3. 카메라 + 어노테이터
# ----------------------------------------------------------------------
import omni.replicator.core as rep

dist = float(max(gt) * 6.0 + 1.0)
cam_pos = (center[0] + dist * 0.6, center[1] - dist * 0.6, center[2] + dist * 0.7)

camera = rep.create.camera(position=cam_pos, look_at=tuple(float(v) for v in center))
rp = rep.create.render_product(camera, RES)
annot = rep.AnnotatorRegistry.get_annotator("bounding_box_3d")
annot.attach(rp)
print("[3] 카메라 %s  ->  %s" % (tuple(round(v, 2) for v in cam_pos),
                                tuple(round(float(v), 2) for v in center)))


# ----------------------------------------------------------------------
# 4. 렌더 -> 결과 읽기
# ----------------------------------------------------------------------
def dims_from_row(row, names):
    def g(k):
        return float(row[names.index(k)]) if isinstance(row, tuple) else float(row[k])
    ext = np.array([g("x_max") - g("x_min"),
                    g("y_max") - g("y_min"),
                    g("z_max") - g("z_min")])
    T = np.array(row["transform"]).reshape(4, 4)
    scale = np.array([np.linalg.norm(T[0, :3]),
                      np.linalg.norm(T[1, :3]),
                      np.linalg.norm(T[2, :3])])
    return ext * scale, T[3, :3]


async def run():
    for _ in range(3):
        await rep.orchestrator.step_async()

    data = annot.get_data()
    rows = data.get("data", [])
    info = data.get("info", {})

    if len(rows) == 0:
        print("[4] 검출 0 건")
        print("    - 카메라 시야에 상자가 없거나, 라벨이 안 붙었을 수 있습니다")
        print("    - info:", info)
        return

    names = list(rows.dtype.names) if hasattr(rows, "dtype") else []
    print("[4] 검출 %d 건   필드: %s" % (len(rows), names))

    for i, row in enumerate(rows):
        d, pos = dims_from_row(row, names)
        d_sorted = np.sort(d)[::-1]
        gt_sorted = np.sort(gt)[::-1]
        err = (d_sorted - gt_sorted) * 1000.0
        print("  [%d] 측정 L%.4f W%.4f H%.4f" % (i, d[0], d[1], d[2]))
        print("      중심 (%.3f, %.3f, %.3f)" % (pos[0], pos[1], pos[2]))
        print("      오차 %+.2f / %+.2f / %+.2f mm (크기순 정렬 비교)"
              % (err[0], err[1], err[2]))

    print("[5] 완료")


asyncio.ensure_future(run())