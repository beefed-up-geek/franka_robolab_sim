"""전선 자연 초기화 — 캡슐 체인을 물리로 정착시켜 곡선을 굽는다.

현재의 나선 배치를 초기 조건으로 쓰고(코일 메모리), 양 끝(소켓 출구·커넥터
인입부)만 키네마틱으로 고정한 채 중력·마찰로 이완시킨다. 결과 폴리라인을
JSON 으로 내보낸다.
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import json
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema
import omni.usd
from isaacsim.core.api import SimulationContext

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

# 지면(상판) — z=0 평면
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
UsdGeom.XformCommonAPI(ground.GetPrim()).SetTranslate(Gf.Vec3d(0.5, 0.1, -0.05))
UsdGeom.XformCommonAPI(ground.GetPrim()).SetScale(Gf.Vec3f(3.0, 3.0, 0.1))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
mat = UsdPhysics.MaterialAPI.Apply(ground.GetPrim())
mat.CreateStaticFrictionAttr(0.9)
mat.CreateDynamicFrictionAttr(0.85)

def smooth(path, it=3):
    p = np.array(path)
    for _ in range(it):
        q = p.copy()
        q[1:-1] = 0.25*p[:-2] + 0.5*p[1:-1] + 0.25*p[2:]
        p = q
    return p

def seg(p0, p1, n=16):
    return np.linspace(p0, p1, n)

def spiral(cx, cy, r0, r1, a0_deg, sweep_deg, z, n=170):
    ts = np.linspace(0, 1, n)
    rs = r0 + (r1 - r0) * ts
    aa = np.radians(a0_deg + sweep_deg * ts)
    return np.stack([cx + rs*np.cos(aa), cy + rs*np.sin(aa), np.full(n, z)], axis=1)

def resample(path, step):
    d = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(d)])
    L = cum[-1]
    n = int(L / step)
    ts = np.linspace(0, L, n)
    out = np.stack([np.interp(ts, cum, path[:, k]) for k in range(3)], axis=1)
    return out

RED = smooth(np.vstack([
    seg((0.49, 0.348, 0.095), (0.46, 0.295, 0.02), 14),
    spiral(0.38, 0.16, 0.115, 0.075, 70, -540, 0.02),
    seg((0.38 + 0.075*np.cos(np.radians(-470)), 0.16 + 0.075*np.sin(np.radians(-470)), 0.02),
        (0.433, -0.012, 0.025), 14),
    seg((0.433, -0.012, 0.025), (0.446, -0.02, 0.040), 8),
]))
BLK = smooth(np.vstack([
    seg((0.51, 0.348, 0.095), (0.55, 0.30, 0.02), 14),
    spiral(0.63, 0.14, 0.115, 0.075, 110, 520, 0.02),
    seg((0.63 + 0.075*np.cos(np.radians(630)), 0.14 + 0.075*np.sin(np.radians(630)), 0.02),
        (0.532, -0.088, 0.025), 14),
    seg((0.532, -0.088, 0.025), (0.526, -0.10, 0.040), 8),
]))

LINK = 0.02
R = 0.006

def build_chain(name, path):
    pts = resample(path, LINK)
    n = len(pts) - 1
    links = []
    for i in range(n):
        a, b = pts[i], pts[i+1]
        mid = (a + b) / 2
        d = b - a
        L = np.linalg.norm(d)
        d /= L
        # 캡슐 축 X 를 d 로 회전
        x = np.array([1.0, 0, 0])
        v = np.cross(x, d)
        c = float(np.dot(x, d))
        if np.linalg.norm(v) < 1e-8:
            q = Gf.Quatf(1, 0, 0, 0) if c > 0 else Gf.Quatf(0, 0, 0, 1)
        else:
            s = np.sqrt((1 + c) * 2)
            q = Gf.Quatf(float(s / 2), *(v / s).tolist())
        cap = UsdGeom.Capsule.Define(stage, f"/World/{name}/link_{i:03d}")
        cap.CreateAxisAttr("X")
        cap.CreateRadiusAttr(R)
        cap.CreateHeightAttr(max(LINK - 2*R, 0.004))
        api = UsdGeom.XformCommonAPI(cap.GetPrim())
        api.SetTranslate(Gf.Vec3d(*mid))
        cap.GetPrim().GetAttribute("xformOp:orient").Set(q) if cap.GetPrim().GetAttribute("xformOp:orient") else None
        xf = UsdGeom.Xformable(cap.GetPrim())
        try:
            xf.AddOrientOp().Set(q)
        except Exception:
            pass
        UsdPhysics.RigidBodyAPI.Apply(cap.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
        massapi = UsdPhysics.MassAPI.Apply(cap.GetPrim())
        massapi.CreateMassAttr(0.004)
        pb = PhysxSchema.PhysxRigidBodyAPI.Apply(cap.GetPrim())
        pb.CreateLinearDampingAttr(4.0)
        pb.CreateAngularDampingAttr(4.0)
        links.append(cap.GetPrim())
    # 관절
    for i in range(n - 1):
        j = UsdPhysics.SphericalJoint.Define(stage, f"/World/{name}/joint_{i:03d}")
        j.CreateBody0Rel().SetTargets([links[i].GetPath()])
        j.CreateBody1Rel().SetTargets([links[i+1].GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(LINK/2, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-LINK/2, 0, 0))
        j.CreateConeAngle0LimitAttr(35.0)
        j.CreateConeAngle1LimitAttr(35.0)
    # 양 끝 고정
    for prim in (links[0], links[-1]):
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    return links

chains = {"red": build_chain("chain_red", RED), "black": build_chain("chain_black", BLK)}

sim = SimulationContext(physics_dt=1/240.0, rendering_dt=1/60.0, backend="numpy", device="cpu")
sim.initialize_physics()
for step in range(1600):
    sim.step(render=False)
print("정착 완료", flush=True)

out = {}
for name, links in chains.items():
    pts = []
    for prim in links:
        m = omni.usd.get_world_transform_matrix(prim)
        t = m.ExtractTranslation()
        pts.append([round(t[0], 4), round(t[1], 4), round(t[2], 4)])
    out[name] = pts
json.dump(out, open("/tmp/wire_settled.json", "w"))
print("링크 수:", {k: len(v) for k, v in out.items()}, flush=True)
app.close()
