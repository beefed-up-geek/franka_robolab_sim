#!/usr/bin/env python3
"""기존 통조림 캔 에셋을 변형해 "파열된" 짝을 만든다.

HOPE 데이터셋 캔들(corn_can 등)은 사진 텍스처가 입혀진 스캔 메시라, 절차적으로 다시
그리면 라벨이 사라진다. 그래서 새로 그리지 않고 **원본 정점을 직접 밀어서** 결함을
만든다. 텍스처·UV·재질이 그대로라 정상품과 파열품이 말 그대로 같은 캔이 되고,
정책이 라벨 그림이 아니라 결함을 보고 판단하게 된다.

    1. 위아래 뚜껑을 볼록하게 밀어 올린다   — 내부 가스 압력
    2. 옆면 한 곳을 안으로 눌러 찌그러뜨린다 — 취급 중 손상
    3. 윗뚜껑의 한 구간을 통째로 지운다      — 파열, 내용물 노출

지운 자리로 배경이 비치지 않게 안쪽 벽과 내용물 면을 따로 붙인다. 이 둘만 원본에
없는 메시라 텍스처 없이 단색 재질을 쓴다.

pxr(USD 파이썬)이 필요해서 컨테이너 안에서 돌려야 한다.

    PYTHONPATH=<omni.usd.libs>/pxr 가 잡힌 상태에서
    /isaac-sim/python.sh tools/burst_from_usd.py <출력폴더> <원본.usd> ...
"""
from __future__ import annotations

import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

# ── 결함 파라미터 ─────────────────────────────────────────────────────────
# 캔 높이에 비례시킨다. 참치캔(33mm)에 통조림(58mm)과 같은 17mm 를 부풀리면
# 캔이 아니라 공이 된다.
TOP_BULGE_RATIO = 0.30      # 윗뚜껑이 부푸는 높이 / 캔 높이
BOT_BULGE_RATIO = 0.0       # 아랫뚜껑 — 2026-08-21 부터 **평평하게** 둔다.
# 아래가 볼록하면 캔이 벨트에서 비스듬히 기울어 파지 자세가 학습 분포를
# 벗어난다 (사용자 지시: 변별은 시각 단서로만, 파지 난도는 정상품과 동일하게).
# 파열 단서는 윗뚜껑 부풂·옆면 찌그러짐·뜯긴 구멍 셋으로 충분하다.
BELLY_SWELL = 0.0022        # 몸통이 배부른 정도 [m]

DENT_C = math.radians(205)  # 찌그러진 방향 (찢어진 쪽 반대편)
DENT_SIG_T = math.radians(42)
DENT_DEPTH = 0.009
DENT_Z, DENT_SIG_Z = 0.40, 0.28

TEAR_C = math.radians(25)   # 뜯겨 나간 구간의 중심 방향
TEAR_HALF = math.radians(72)

# 뚜껑을 "높이 상위 몇 %" 로 잡으면 안 된다. 납작한 참치캔은 뚜껑이 테두리보다
# 오목하게 들어가 있어서, 그렇게 잡으면 뚜껑 대신 몸통 윗부분이 잘려 나간다
# (실제로 그렇게 만들었다가 참치·양송이만 멀쩡한 채로 나왔다).
# 그래서 반지름별 최고 z 프로파일을 만들어 "그 반지름에서 가장 높은 면" 을 뚜껑으로 본다.
LID_TOL_RATIO = 0.16        # 프로파일에서 이만큼(캔 높이 대비) 아래까지 뚜껑으로 친다
LID_TOL_MIN = 0.004         # 최소 허용치 [m]
RIM_RHO = 0.955             # 이 반지름 비율보다 바깥은 시밍 테두리라 건드리지 않는다
PROF_BINS = 48


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def tear_open(theta: float, rho: float) -> bool:
    """이 방향·반지름이 뜯겨 나간 구간인지. 경계를 흔들어 찢긴 자국으로 보이게 한다."""
    jag = math.radians(6.0) * math.sin(6.3 * theta + 1.1) \
        + math.radians(3.0) * math.sin(11.7 * theta + 0.4)
    # 중심으로 갈수록 조금 넓어져야 구멍이 하나로 이어진다
    half = TEAR_HALF * (1.0 + 0.12 * (1.0 - min(1.0, rho))) + jag
    return abs(wrap(theta - TEAR_C)) <= half


def find_mesh(stage: Usd.Stage) -> UsdGeom.Mesh:
    for p in stage.Traverse():
        if p.IsA(UsdGeom.Mesh):
            return UsdGeom.Mesh(p)
    raise SystemExit("메시를 찾지 못했다")


def lid_dome(rho: float) -> float:
    """뚜껑 돔의 높이 계수. 테두리(RIM_RHO)에서 정확히 0 이라 이음매가 끊기지 않는다."""
    return max(0.0, math.cos(min(1.0, rho / RIM_RHO) * math.pi / 2)) ** 0.75


def deform(points, zmin, zmax, rmax):
    """정점을 밀어 결함을 만든다. 원본 리스트는 건드리지 않는다.

    뚜껑은 "원래 표면을 위로 민다" 가 아니라 **테두리를 기준으로 돔을 다시 그린다.**
    HOPE 캔들은 뚜껑이 테두리보다 오목하게 들어가 있어서, 원래 표면을 기준으로
    밀면 그 오목한 만큼을 까먹어 부풂이 절반쯤만 반영된다(옥수수캔이 17mm 대신
    11mm 만 올라갔다). 테두리 높이에 돔을 얹는 방식이면 캔마다 같은 양이 나온다.

    각 점은 "그 반지름에서의 목표 돔 높이 − 현재 윗면 높이" 만큼 옮긴다. 통째로
    대입하지 않고 차이만 더하므로 뚜껑에 새겨진 요철은 남는다.
    """
    h = zmax - zmin
    tol = max(LID_TOL_MIN, LID_TOL_RATIO * h)
    prof_top = surface_profile(points, rmax, top=True)
    prof_bot = surface_profile(points, rmax, top=False)
    z_rim_top = prof_lerp(prof_top, RIM_RHO)
    z_rim_bot = prof_lerp(prof_bot, RIM_RHO)
    # 부푸는 양은 바운딩박스가 아니라 **테두리 사이 몸통 높이**를 기준으로 잡는다.
    # 바운딩박스는 스캔마다 라벨 가장자리 같은 잡점이 섞여 캔별로 10% 넘게 흔들린다
    # (같은 58mm 캔인데 옥수수 83mm, 완두콩 90mm 로 갈렸다).
    # 스캔에 라벨 가장자리 같은 잡점이 섞이면 테두리 표본이 어긋나므로 전체 높이로 막는다.
    h_body = max(1e-6, min(h, z_rim_top - z_rim_bot))
    top_bulge, bot_bulge = TOP_BULGE_RATIO * h_body, BOT_BULGE_RATIO * h_body

    out = []
    for x, y, z in points:
        r = math.hypot(x, y)
        th = math.atan2(y, x)
        t = (z - zmin) / h if h > 0 else 0.5
        rho = r / rmax if rmax > 0 else 0.0

        # 몸통 — 배부름 + 찌그러짐. 뚜껑 중앙(rho≈0)은 건드리지 않는다.
        dr = BELLY_SWELL * math.sin(math.pi * t) * rho
        dth = wrap(th - DENT_C)
        dr -= DENT_DEPTH * rho * math.exp(
            -(dth / DENT_SIG_T) ** 2 - ((t - DENT_Z) / DENT_SIG_Z) ** 2)
        nr = max(0.0, r + dr)

        nz = z
        if rho < RIM_RHO:
            top_s, bot_s = prof_lerp(prof_top, rho), prof_lerp(prof_bot, rho)
            if z >= top_s - tol:
                # 겉면에서 얼마나 파여 있었는지(rel)를 유지한 채 목표 돔에 얹는다.
                # rel 을 음수로 두면 프로파일 보간 오차만큼 돔 위로 삐져나가
                # 캔이 제각각 높아진다(완두콩만 7mm 더 높았다).
                nz = (z_rim_top + top_bulge * lid_dome(rho)) - max(0.0, top_s - z)
            elif z <= bot_s + tol:
                nz = (z_rim_bot - bot_bulge * lid_dome(rho)) + max(0.0, z - bot_s)

        out.append((nr * math.cos(th), nr * math.sin(th), nz))
    return out


def surface_profile(points, rmax, *, top: bool):
    """반지름 구간별 최고(또는 최저) z. 뚜껑이 오목하든 볼록하든 겉면을 따라간다."""
    prof = [None] * PROF_BINS
    for x, y, z in points:
        b = min(PROF_BINS - 1, int(math.hypot(x, y) / rmax * PROF_BINS)) if rmax > 0 else 0
        if prof[b] is None or (z > prof[b] if top else z < prof[b]):
            prof[b] = z
    # 점이 하나도 없는 구간은 이웃값으로 메운다
    known = [z for z in prof if z is not None]
    fill = (max(known) if top else min(known)) if known else 0.0
    for i in range(PROF_BINS):
        if prof[i] is None:
            prof[i] = prof[i - 1] if i and prof[i - 1] is not None else fill
    return prof


def top_profile(points, rmax):
    return surface_profile(points, rmax, top=True)


def prof_at(prof: list, rho: float) -> float:
    return prof[min(len(prof) - 1, max(0, int(rho * len(prof))))]


def prof_lerp(prof: list, rho: float) -> float:
    """구간 사이를 선형으로 이어 준다. 계단으로 두면 변형에 단차가 생긴다."""
    n = len(prof)
    u = max(0.0, min(n - 1e-6, rho * n)) - 0.5      # 구간 중심 기준
    i = int(math.floor(u))
    if i < 0:
        return prof[0]
    if i >= n - 1:
        return prof[n - 1]
    f = u - i
    return prof[i] * (1 - f) + prof[i + 1] * f


def cut_tear(points, counts, indices, zmin, zmax, rmax):
    """윗뚜껑의 뜯긴 구간에 속한 면을 지운다.

    반환: (남은 counts, 남은 indices, 남긴 코너 위치, 찾은 뚜껑 면 수, 지운 면 수)
    """
    h = zmax - zmin
    prof = top_profile(points, rmax)
    tol = max(LID_TOL_MIN, LID_TOL_RATIO * h)
    new_counts, new_idx, kept_corners = [], [], []
    lid = dropped = 0
    c = 0
    for n in counts:
        corners = indices[c:c + n]
        cx = sum(points[i][0] for i in corners) / n
        cy = sum(points[i][1] for i in corners) / n
        cz = sum(points[i][2] for i in corners) / n
        rho = math.hypot(cx, cy) / rmax if rmax > 0 else 0.0
        # 그 반지름에서 가장 높은 면에 붙어 있으면 뚜껑이다
        is_lid = rho < RIM_RHO and cz >= prof_at(prof, rho) - tol
        if is_lid:
            lid += 1
        if is_lid and tear_open(math.atan2(cy, cx), rho):
            dropped += 1
        else:
            new_counts.append(n)
            new_idx.extend(corners)
            kept_corners.extend(range(c, c + n))
        c += n
    return new_counts, new_idx, kept_corners, lid, dropped, prof


def add_patch(stage, root: str, name: str, rmax: float, z_top: float, h: float,
              mat_path: str, kind: str) -> None:
    """뚫린 자리를 막는 면. kind 는 'wall'(안쪽 벽) 또는 'contents'(내용물).

    z_top 은 뚜껑 테두리 높이다. 부푼 돔의 꼭대기를 쓰면 납작한 캔에서 벽이
    캔 밖으로 솟는다. 깊이도 캔 높이에 비례시켜야 바닥을 뚫지 않는다.
    """
    seg = 72
    ang = [2 * math.pi * i / seg for i in range(seg)]
    pts, counts, idx = [], [], []

    if kind == "wall":
        depth = min(0.016, 0.42 * h)
        for a in ang:
            pts.append(Gf.Vec3f(rmax * 0.995 * math.cos(a), rmax * 0.995 * math.sin(a), z_top))
        for a in ang:
            pts.append(Gf.Vec3f(rmax * 0.93 * math.cos(a), rmax * 0.93 * math.sin(a), z_top - depth))
        for i in range(seg):
            k = (i + 1) % seg
            counts.append(4)
            idx.extend([i, k, seg + k, seg + i])
    else:
        z = z_top - min(0.017, 0.45 * h)
        pts.append(Gf.Vec3f(0.0, 0.0, z - 0.003))
        for a in ang:
            rr = rmax * 0.90 + 0.0015 * math.sin(5 * a)
            pts.append(Gf.Vec3f(rr * math.cos(a), rr * math.sin(a),
                                z + 0.0015 * math.cos(4 * a) + 0.0010 * math.sin(7 * a)))
        for i in range(seg):
            k = (i + 1) % seg
            counts.append(3)
            idx.extend([1 + i, 1 + k, 0])

    mesh = UsdGeom.Mesh.Define(stage, f"{root}/{name}")
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateSubdivisionSchemeAttr("none")
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(min(xs), min(ys), min(zs)),
                                         Gf.Vec3f(max(xs), max(ys), max(zs))]))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        UsdShade.Material(stage.GetPrimAtPath(mat_path)))


def plain_material(stage, path: str, rgb, metallic: float, rough: float) -> str:
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(rough)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(out)
    mat.CreateDisplacementOutput("mdl").ConnectToSource(out)
    mat.CreateVolumeOutput("mdl").ConnectToSource(out)
    return path


def burst(src_path: str, out_dir: str) -> str:
    src = Usd.Stage.Open(src_path)
    src_mesh = find_mesh(src)
    src_root = src.GetDefaultPrim()
    name = src_root.GetName()

    pts = [tuple(p) for p in src_mesh.GetPointsAttr().Get()]
    counts = list(src_mesh.GetFaceVertexCountsAttr().Get())
    idx = list(src_mesh.GetFaceVertexIndicesAttr().Get())

    zmin = min(p[2] for p in pts)
    zmax = max(p[2] for p in pts)
    rmax = max(math.hypot(p[0], p[1]) for p in pts)

    moved = deform(pts, zmin, zmax, rmax)
    new_counts, new_idx, kept_corners, lid_faces, dropped, prof = \
        cut_tear(moved, counts, idx, zmin, zmax, rmax)
    if dropped == 0:
        raise SystemExit(f"{name}: 뚜껑을 하나도 지우지 못했다 — 뚜껑 판정이 틀렸다")
    lid_rim_z = prof_lerp(prof, 0.92)   # 뚜껑 테두리 높이. 막음면은 여기서 시작한다.

    # ── 새 스테이지 ──────────────────────────────────────────────────
    out_path = os.path.join(out_dir, f"{name}_burst.usd")
    if os.path.exists(out_path):
        os.remove(out_path)
    dst = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(dst, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(dst, 1.0)
    root_path = f"/{name}_burst"
    root = UsdGeom.Xform.Define(dst, root_path)
    dst.SetDefaultPrim(root.GetPrim())

    # 원본 재질(텍스처 포함)을 통째로 복사한 뒤 텍스처 경로만 새 위치 기준으로 고친다
    src_layer = src.GetRootLayer()
    # 재질이 여럿이면(normal_can 계열: Steel + CanLabel) **텍스처를 가진 것**을
    # 고른다. "처음 발견한 재질" 은 순회 순서에 따라 무지 스틸이 걸려 라벨이
    # 사라진다 (실측: 재생성한 파열 캔이 전부 민무늬가 됐다).
    mat_root = None
    for p in src.Traverse():
        if not p.IsA(UsdShade.Material):
            continue
        if mat_root is None:
            mat_root = p
        has_tex = any(
            isinstance(a.Get(), Sdf.AssetPath)
            and a.Get().path.endswith((".png", ".jpg", ".jpeg"))
            for q in Usd.PrimRange(p) for a in q.GetAttributes())
        if has_tex:
            mat_root = p
            break
    if mat_root is None:
        raise SystemExit(f"{name}: 재질을 찾지 못했다")
    dst_mat_path = f"{root_path}/Looks/{mat_root.GetName()}"
    dst.DefinePrim(f"{root_path}/Looks", "Scope")
    Sdf.CreatePrimInLayer(dst.GetRootLayer(), Sdf.Path(dst_mat_path))
    Sdf.CopySpec(src_layer, mat_root.GetPath(), dst.GetRootLayer(), Sdf.Path(dst_mat_path))

    src_dir = os.path.dirname(os.path.abspath(src_path))
    for p in Usd.PrimRange(dst.GetPrimAtPath(dst_mat_path)):
        for a in p.GetAttributes():
            v = a.Get()
            if isinstance(v, Sdf.AssetPath) and v.path.endswith((".png", ".jpg", ".jpeg")):
                abs_tex = os.path.normpath(os.path.join(src_dir, v.path))
                a.Set(Sdf.AssetPath(os.path.relpath(abs_tex, os.path.abspath(out_dir))))

    # ── 변형된 본체 ──────────────────────────────────────────────────
    mesh = UsdGeom.Mesh.Define(dst, f"{root_path}/shell")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in moved]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(new_counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(new_idx))
    mesh.CreateSubdivisionSchemeAttr("none")
    if src_mesh.GetNormalsAttr().HasAuthoredValue():
        pass    # 변형 후에는 원본 법선이 맞지 않는다. 지우고 렌더러가 계산하게 둔다.

    # UV. faceVarying 이면 지운 면의 코너도 같이 지워야 한다.
    src_pv = UsdGeom.PrimvarsAPI(src_mesh.GetPrim())
    dst_pv = UsdGeom.PrimvarsAPI(mesh.GetPrim())
    for pv in src_pv.GetPrimvars():
        if pv.GetName() not in ("primvars:st", "primvars:displayColor", "primvars:displayOpacity"):
            continue
        interp, vals = pv.GetInterpolation(), pv.Get()
        if vals is None:
            continue        # 선언만 되어 있고 값이 없는 primvar (displayColor 등)
        indices = pv.GetIndices() if pv.IsIndexed() else None
        if interp == UsdGeom.Tokens.faceVarying:
            if indices is not None:
                indices = Vt.IntArray([indices[c] for c in kept_corners])
            else:
                vals = type(vals)([vals[c] for c in kept_corners])
        np_ = dst_pv.CreatePrimvar(pv.GetName().split(":", 1)[1],
                                   pv.GetTypeName(), interp)
        np_.Set(vals)
        if indices is not None:
            np_.SetIndices(indices)

    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        UsdShade.Material(dst.GetPrimAtPath(dst_mat_path)))

    # ── 뚫린 자리 막음 ───────────────────────────────────────────────
    torn = plain_material(dst, f"{root_path}/Looks/TornEdge", (0.80, 0.81, 0.83), 0.92, 0.22)
    cont = plain_material(dst, f"{root_path}/Looks/Contents", (0.40, 0.26, 0.12), 0.0, 0.88)
    h_can = zmax - zmin
    add_patch(dst, root_path, "torn_inner", rmax, lid_rim_z, h_can, torn, "wall")
    add_patch(dst, root_path, "contents", rmax, lid_rim_z, h_can, cont, "contents")

    # ── 원점을 바운딩박스 중심으로 ───────────────────────────────────
    # franka_env/conveyor.py 가 반높이를 재서 벨트 안착 높이를 계산하는데
    # 그 계산이 원점=중심을 전제한다.
    lo = min(p[2] for p in moved)
    hi = max(p[2] for p in moved)
    dz = -(lo + hi) / 2
    if abs(dz) > 1e-9:
        for prim in (mesh.GetPrim(), dst.GetPrimAtPath(f"{root_path}/torn_inner"),
                     dst.GetPrimAtPath(f"{root_path}/contents")):
            g = UsdGeom.Mesh(prim)
            g.CreatePointsAttr(Vt.Vec3fArray(
                [Gf.Vec3f(p[0], p[1], p[2] + dz) for p in g.GetPointsAttr().Get()]))
    height = hi - lo

    xs = [p[0] for p in moved]; ys = [p[1] for p in moved]
    mesh.CreateExtentAttr(Vt.Vec3fArray([
        Gf.Vec3f(min(xs), min(ys), lo + dz), Gf.Vec3f(max(xs), max(ys), hi + dz)]))

    # ── 물리 ─────────────────────────────────────────────────────────
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(root.GetPrim())
    src_mass = UsdPhysics.MassAPI(src_root).GetMassAttr().Get() \
        if src_root.HasAPI(UsdPhysics.MassAPI) else None
    mass_api.CreateMassAttr(float(src_mass) if src_mass else 0.4)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    # 찌그러진 메시는 볼록하지 않아 동적 강체에서는 볼록 근사가 필요하다
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")

    dst.SetMetadata("comment",
                    f"{name} 을 tools/burst_from_usd.py 로 변형한 파열품. "
                    "원본 텍스처·UV 를 그대로 쓴다. 형상을 바꾸려면 이 파일이 아니라 "
                    "생성기를 고쳐 다시 만들 것.")
    dst.GetRootLayer().Save()

    print(f"{name}_burst.usd  높이 {height * 1000:5.1f}mm "
          f"(벨트 안착 z={0.200 + height / 2:.4f})  "
          f"뚜껑면 {lid_faces} 중 {dropped} 뜯김 ({dropped / max(1, lid_faces):.0%}), "
          f"전체 {len(counts)}→{len(new_counts)}")
    return out_path


if __name__ == "__main__":
    out_dir, srcs = sys.argv[1], sys.argv[2:]
    os.makedirs(out_dir, exist_ok=True)
    for s in srcs:
        burst(s, out_dir)
