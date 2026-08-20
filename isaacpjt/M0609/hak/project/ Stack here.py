"""
현재 스테이지 유지 : 스테이지에 있는 Cube 들을 인식 -> 팔레트에 적재

Script Editor:
    exec(open("/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project/stack_here.py").read())
"""

import asyncio
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom

PALLET_PATH = "/World/Pallet"
PW, PD, PH = 1.10, 1.10, 0.15      # 팔레트 W, D, 데크 두께
PALLET_XY = (0.0, 0.0)             # 팔레트 최소 모서리 위치
GAP = 0.005
LABEL = "box"

EXCLUDE = ("/World/Pallet", "/World/Ground", "/World/GroundPlane",
           "/World/defaultGroundPlane", "/Replicator", "/OmniverseKit")

stage = omni.usd.get_context().get_stage()


# ----------------------------------------------------------------------
def is_box(prim):
    p = str(prim.GetPath())
    if any(p.startswith(e) for e in EXCLUDE):
        return False
    return prim.GetTypeName() in ("Cube", "Mesh")


boxes = [p for p in stage.Traverse() if is_box(p)]
if not boxes:
    raise RuntimeError("Cube 프림을 못 찾았습니다. Create > Shapes > Cube 로 만드세요.")
print("[1] 대상 %d 개" % len(boxes))
for p in boxes:
    print("    ", p.GetPath())


# ----------------------------------------------------------------------
def attach_label(p, name):
    try:
        from isaacsim.core.utils.semantics import add_update_semantics
        add_update_semantics(p, name)
        return "isaacsim"
    except Exception:
        pass
    try:
        from omni.isaac.core.utils.semantics import add_update_semantics
        add_update_semantics(p, name)
        return "omni.isaac"
    except Exception:
        pass
    from pxr import Semantics
    sem = Semantics.SemanticsAPI.Apply(p, "Semantics")
    sem.CreateSemanticTypeAttr().Set("class")
    sem.CreateSemanticDataAttr().Set(name)
    return "pxr"


how = ""
for p in boxes:
    how = attach_label(p, LABEL)
print("[2] 라벨 부착 (%s)" % how)


# ----------------------------------------------------------------------
# 팔레트 (없으면 생성)
# ----------------------------------------------------------------------
if not stage.GetPrimAtPath(PALLET_PATH).IsValid():
    try:
        from isaacsim.core.api.objects import FixedCuboid
    except ImportError:
        from omni.isaac.core.objects import FixedCuboid
    FixedCuboid(
        prim_path=PALLET_PATH, name="pallet",
        position=np.array([PALLET_XY[0] + PW / 2, PALLET_XY[1] + PD / 2, PH / 2]),
        scale=np.array([PW, PD, PH]),
        color=np.array([0.25, 0.35, 0.55]),
    )
    print("[3] 팔레트 생성")
else:
    print("[3] 팔레트 이미 있음")


# ----------------------------------------------------------------------
# 카메라 : 전체가 보이도록 위에서
# ----------------------------------------------------------------------
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
mins, maxs = [], []
for p in boxes:
    r = cache.ComputeWorldBound(p).ComputeAlignedRange()
    mins.append([r.GetMin()[i] for i in range(3)])
    maxs.append([r.GetMax()[i] for i in range(3)])
lo, hi = np.min(mins, axis=0), np.max(maxs, axis=0)
mid = (lo + hi) / 2.0
span = float(np.max(hi - lo)) + 0.5

import omni.replicator.core as rep

cam = rep.create.camera(position=(float(mid[0]), float(mid[1] - span * 1.2),
                                  float(mid[2] + span * 1.4)),
                        look_at=(float(mid[0]), float(mid[1]), float(mid[2])))
rp = rep.create.render_product(cam, (640, 480))
annot = rep.AnnotatorRegistry.get_annotator("bounding_box_3d")
annot.attach(rp)
print("[4] 카메라 준비")


# ----------------------------------------------------------------------
def set_pos(path, pos):
    try:
        from isaacsim.core.prims import SingleXFormPrim as XP
    except ImportError:
        from omni.isaac.core.prims import XFormPrim as XP
    XP(str(path)).set_world_pose(position=np.array(pos))


async def run():
    for _ in range(3):
        await rep.orchestrator.step_async()

    data = annot.get_data()
    rows = data.get("data", [])
    info = data.get("info", {})
    if len(rows) == 0:
        print("[5] 검출 0 건.  info:", info)
        return

    prim_paths = info.get("primPaths", None)
    print("[5] 검출 %d 건" % len(rows))

    items = []
    for i, row in enumerate(rows):
        ext = np.array([float(row["x_max"]) - float(row["x_min"]),
                        float(row["y_max"]) - float(row["y_min"]),
                        float(row["z_max"]) - float(row["z_min"])])
        T = np.array(row["transform"]).reshape(4, 4)
        sc = np.array([np.linalg.norm(T[0, :3]),
                       np.linalg.norm(T[1, :3]),
                       np.linalg.norm(T[2, :3])])
        dims = ext * sc
        ctr = T[3, :3]

        # 프림 매칭 : primPaths 우선, 없으면 중심 최근접
        if prim_paths is not None and i < len(prim_paths):
            path = prim_paths[i]
        else:
            best, bd = None, 1e9
            for p in boxes:
                r = cache.ComputeWorldBound(p).ComputeAlignedRange()
                m = r.GetMidpoint()
                d = np.linalg.norm(np.array([m[0], m[1], m[2]]) - ctr)
                if d < bd:
                    best, bd = p.GetPath(), d
            path = best

        if any(str(path).startswith(e) for e in EXCLUDE):
            continue
        items.append({"path": str(path), "dims": dims})
        print("    %s  L%.3f W%.3f H%.3f" % (path, dims[0], dims[1], dims[2]))

    # ---- 배치 : 왼쪽 아래부터, 줄 차면 다음 줄, 줄 다 차면 다음 층 ----
    items.sort(key=lambda it: -(it["dims"][0] * it["dims"][1] * it["dims"][2]))
    x = y = z = 0.0
    row_d = layer_h = 0.0
    n = 0
    for it in items:
        L, W, H = it["dims"]
        if L > PW or W > PD:
            print("    [skip] %s 팔레트보다 큼" % it["path"])
            continue
        if x + L > PW:
            x = 0.0
            y += row_d + GAP
            row_d = 0.0
        if y + W > PD:
            x = y = 0.0
            z += layer_h + GAP
            row_d = layer_h = 0.0

        pos = (PALLET_XY[0] + x + L / 2,
               PALLET_XY[1] + y + W / 2,
               PH + z + H / 2)
        set_pos(it["path"], pos)
        print("    -> %s  (%.3f, %.3f, %.3f)" % (it["path"], pos[0], pos[1], pos[2]))
        x += L + GAP
        row_d = max(row_d, W)
        layer_h = max(layer_h, H)
        n += 1

    print("[6] 적재 %d 개 완료. Play 로 확인." % n)


asyncio.ensure_future(run())