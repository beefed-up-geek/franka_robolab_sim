"""씬의 공구별 주축(PCA) 방위각을 잰다 — X축 정렬 검증의 수치 기준."""
import math
from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open("/workspace/franka_robolab_sim/env/asset/scenes/task1_handover.usda")
cache = UsdGeom.XformCache(Usd.TimeCode.Default())
for name in ("hammer_7", "cordless_drill", "scissors"):
    root = stage.GetPrimAtPath(f"/world/{name}")
    pts = []
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "Mesh":
            continue
        mesh = UsdGeom.Mesh(prim)
        p = mesh.GetPointsAttr().Get()
        if not p:
            continue
        m = cache.GetLocalToWorldTransform(prim)
        step = max(1, len(p) // 400)          # 표본화 — 정확도엔 충분
        for i in range(0, len(p), step):
            w = m.Transform(Gf.Vec3d(p[i]))
            pts.append((w[0], w[1]))
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0]-mx)**2 for p in pts) / n
    syy = sum((p[1]-my)**2 for p in pts) / n
    sxy = sum((p[0]-mx)*(p[1]-my) for p in pts) / n
    ang = 0.5 * math.atan2(2*sxy, sxx - syy)
    deg = math.degrees(ang)
    # X축 기준 편차 (-90~90)
    print(f"{name}: 주축 {deg:+.1f}° (X축에서 벗어난 각)  표본 {n}")
