#!/usr/bin/env python3
"""작업자 '손 내밀기' 자세 래퍼 USDA 생성 — 팔 관절 델타 회전을 rest 에 합성한다."""
import math
import sys

REST = {  # (w,x,y,z), 로컬 translate
    "R_Clavicle": ((0.6921, -0.0532, -0.1529, 0.7034), (-0.061, 0.234, -0.002)),
    "R_Upperarm": ((0.9898, 0.1425, -0.0031, -0.0074), (-0.000, 0.135, -0.000)),
    "R_Forearm":  ((1.0000, -0.0063, 0.0001, -0.0075), (0.000, 0.276, -0.000)),
    "R_Hand":     ((0.9971, -0.0382, -0.0282, -0.0598), (-0.000, 0.219, -0.000)),
    "L_Clavicle": ((-0.69324, 0.05419, -0.15347, 0.70209), (0.061, 0.2345, -0.0017)),
    "L_Upperarm": ((0.98963, 0.14347, 0.00327, 0.00684), (0.0, 0.1349, 0.0)),
    "L_Forearm":  ((0.99995, -0.00558, -0.00027, 0.00831), (0.0, 0.2756, 0.0)),
    "L_Hand":     ((0.997, -0.03859, 0.02838, 0.06086), (0.0, 0.2201, 0.0)),
}
RBASE = "RL_BoneRoot/Hip/Waist/Spine01/Spine02/R_Clavicle"
LBASE = "RL_BoneRoot/Hip/Waist/Spine01/Spine02/L_Clavicle"
PATHS = {
    "R_Clavicle": RBASE,
    "R_Upperarm": RBASE + "/R_Upperarm",
    "R_Forearm":  RBASE + "/R_Upperarm/R_Forearm",
    "R_Hand":     RBASE + "/R_Upperarm/R_Forearm/R_Hand",
    "L_Clavicle": LBASE,
    "L_Upperarm": LBASE + "/L_Upperarm",
    "L_Forearm":  LBASE + "/L_Upperarm/L_Forearm",
    "L_Hand":     LBASE + "/L_Upperarm/L_Forearm/L_Hand",
}

def qmul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)

def axis_q(axis, deg):
    r = math.radians(deg) / 2
    s = math.sin(r)
    v = {"x": (s, 0, 0), "y": (0, s, 0), "z": (0, 0, s)}[axis]
    return (math.cos(r), *v)

def make(deltas: dict, out: str) -> None:
    joints, rots, trs = [], [], []
    for name in deltas.get("_joints", ("R_Clavicle", "R_Upperarm", "R_Forearm", "R_Hand")):
        q, t = REST[name]
        for axis, deg in deltas.get(name, []):
            q = qmul(q, axis_q(axis, deg))
        joints.append(f'"{PATHS[name]}"')
        rots.append(f"({q[0]:.5f}, {q[1]:.5f}, {q[2]:.5f}, {q[3]:.5f})")
        trs.append(f"({t[0]}, {t[1]}, {t[2]})")
    scales = ", ".join("(1, 1, 1)" for _ in joints)
    text = f'''#usda 1.0
(
    doc = """작업자 — 오른팔을 들어 손을 내민 정지 자세.

    Isaac People 건설작업자에 SkelAnimation 을 얹어 rest(T포즈)의 오른팔 관절만
    덮어쓴다. 목록에 없는 관절은 rest 그대로다. 회전값은 tools/gen_pose.py 가
    rest ⊗ 델타로 계산해 굽는다 — 손으로 만지지 말고 생성기를 고칠 것.
    """
    defaultPrim = "Root"
    metersPerUnit = 1
    upAxis = "Y"
)

def "Root" (
    prepend references = @./worker/male_adult_construction_01_new.usd@</Root>
)
{{
    over "male_adult_construction_01"
    {{
        over "ManRoot"
        {{
            over "male_adult_construction_01" (
                prepend apiSchemas = ["SkelBindingAPI"]
            )
            {{
                rel skel:animationSource = </Root/male_adult_construction_01/ManRoot/male_adult_construction_01/ReceivePose>

                def SkelAnimation "ReceivePose"
                {{
                    uniform token[] joints = [{", ".join(joints)}]
                    quatf[] rotations = [{", ".join(rots)}]
                    float3[] translations = [{", ".join(trs)}]
                    half3[] scales = [{scales}]
                }}
            }}
        }}
    }}
}}
'''
    open(out, "w").write(text)
    print("생성:", out)

if __name__ == "__main__":
    # 후보: 이름=축각도리스트  예) A: Upperarm z-70
    cand = sys.argv[1]
    table = {
        # 손 내밀기 (핸드오버) — 오른팔 앞 70° + 아래 25°
        "reach": {"R_Upperarm": [("x", -70), ("z", 25)],
                  "R_Forearm": [("x", -10)], "R_Hand": [("z", -15)]},
        # 차렷 — 양팔 내림. T포즈에서 어깨를 몸통 쪽으로 75° 내린다.
        # (오른팔은 z+, 왼팔은 미러라 z- 방향이 "아래" — 렌더로 확정)
        "attention": {"_joints": ("R_Upperarm", "R_Forearm", "L_Upperarm", "L_Forearm"),
                      "R_Upperarm": [("z", 75)], "R_Forearm": [("z", 5)],
                      "L_Upperarm": [("z", -75)], "L_Forearm": [("z", -5)]},
        # 왼손 대각선 내밀기 — 왼팔을 앞으로 70° 스윙(x, 오른팔의 미러 부호)
        # 하고 40° 아래로 기울인다. 오른팔은 차렷 그대로.
        "reach_left": {"_joints": ("R_Upperarm", "R_Forearm",
                                   "L_Upperarm", "L_Forearm", "L_Hand"),
                       "R_Upperarm": [("z", 75)], "R_Forearm": [("z", 5)],
                       "L_Upperarm": [("x", 70), ("z", -40)],
                       "L_Forearm": [("x", 10)], "L_Hand": [("z", 15)]},
    }
    make(table[cand], sys.argv[2])
