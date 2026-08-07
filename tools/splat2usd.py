"""SAM3D 가우시안 스플랫 PLY → 컬러 메시 USD 변환.

스플랫 점(위치·법선·SH0색·불투명도) → 포아송 재구성 → 밀도 트림 →
최대 연결 성분 → 단순화 → 점군 색 전사 → 실물 스케일 정규화(장축 지정 길이,
바닥 z=0, 중심 xy=0) → UsdGeomMesh(displayColor vertex) 저장.
"""
import sys
import numpy as np
from plyfile import PlyData
import open3d as o3d
from pxr import Usd, UsdGeom, Gf, Vt

src, dst, target_long = sys.argv[1], sys.argv[2], float(sys.argv[3])
rot = sys.argv[4] if len(sys.argv) > 4 else "none"   # 축 보정: none|x90|x-90|z90 ...

ply = PlyData.read(src)
v = ply["vertex"]
pts = np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float64)
nrm = np.stack([v["nx"], v["ny"], v["nz"]], axis=1).astype(np.float64)
C0 = 0.28209479177
rgb = np.clip(0.5 + C0 * np.stack([v["f_dc_0"], v["f_dc_1"], v["f_dc_2"]], axis=1), 0, 1)
op = 1.0 / (1.0 + np.exp(-np.asarray(v["opacity"], dtype=np.float64)))
keep = op > 0.3
pts, nrm, rgb = pts[keep], nrm[keep], rgb[keep]
print(f"점 {keep.sum()}/{len(keep)} (불투명도 필터)", flush=True)

pc = o3d.geometry.PointCloud()
pc.points = o3d.utility.Vector3dVector(pts)
pc.colors = o3d.utility.Vector3dVector(rgb)
if np.linalg.norm(nrm, axis=1).max() > 1e-6:
    pc.normals = o3d.utility.Vector3dVector(nrm)
pc = pc.voxel_down_sample(voxel_size=float(np.ptp(pts, axis=0).max()) / 256.0)
print(f"다운샘플 후 {len(pc.points)}점", flush=True)
if not pc.has_normals() or np.linalg.norm(np.asarray(pc.normals), axis=1).max() < 1e-6:
    # 스플랫 PLY 의 법선은 0 자리채움이 일반적 — 직접 추정 후 일관 정렬
    print("법선 추정 중…", flush=True)
    r = float(np.ptp(np.asarray(pc.points), axis=0).max()) / 64.0
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=r, max_nn=40))
    pc.orient_normals_consistent_tangent_plane(30)

mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pc, depth=9)
dens = np.asarray(dens)
mesh.remove_vertices_by_mask(dens < np.quantile(dens, 0.04))
tri_c, cnt, _ = mesh.cluster_connected_triangles()
tri_c, cnt = np.asarray(tri_c), np.asarray(cnt)
mesh.remove_triangles_by_mask(cnt[tri_c] < cnt.max())
mesh.remove_unreferenced_vertices()
mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=60000)
mesh.remove_unreferenced_vertices()
print(f"메시 {len(mesh.vertices)}정점 {len(mesh.triangles)}삼각형", flush=True)

# 색 전사 (최근접 점)
kd = o3d.geometry.KDTreeFlann(pc)
mv = np.asarray(mesh.vertices)
src_rgb = np.asarray(pc.colors)
out_rgb = np.empty((len(mv), 3))
for i, p in enumerate(mv):
    _, idx, _ = kd.search_knn_vector_3d(p, 1)
    out_rgb[i] = src_rgb[idx[0]]

# 축 보정 + 정규화
R = {"none": np.eye(3),
     "x90": o3d.geometry.get_rotation_matrix_from_xyz([np.pi/2, 0, 0]),
     "x-90": o3d.geometry.get_rotation_matrix_from_xyz([-np.pi/2, 0, 0]),
     "z90": o3d.geometry.get_rotation_matrix_from_xyz([0, 0, np.pi/2]),
     "z180": o3d.geometry.get_rotation_matrix_from_xyz([0, 0, np.pi]),
     }[rot]
mv = mv @ R.T
ext = np.ptp(mv, axis=0)
s = target_long / ext.max()
mv = mv * s
mv[:, 0] -= (mv[:, 0].max() + mv[:, 0].min()) / 2
mv[:, 1] -= (mv[:, 1].max() + mv[:, 1].min()) / 2
mv[:, 2] -= mv[:, 2].min()
print(f"정규화 크기: {np.round(np.ptp(mv, axis=0), 4)}", flush=True)

stage = Usd.Stage.CreateNew(dst)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
xf = UsdGeom.Xform.Define(stage, "/asset")
stage.SetDefaultPrim(xf.GetPrim())
m = UsdGeom.Mesh.Define(stage, "/asset/mesh")
m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in mv]))
tris = np.asarray(mesh.triangles)
m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tris)))
m.CreateFaceVertexIndicesAttr(Vt.IntArray(tris.flatten().tolist()))
m.CreateSubdivisionSchemeAttr("none")
cp = m.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
cp.Set(Vt.Vec3fArray([Gf.Vec3f(*c) for c in out_rgb]))
stage.GetRootLayer().Save()
print("저장:", dst, flush=True)
