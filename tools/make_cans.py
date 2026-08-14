#!/usr/bin/env python3
"""통조림 캔 USD 생성기 — 정상품과 파열품 한 쌍.

식품 물류에서 팽창관(swollen can)은 내용물이 부패해 가스가 찬 불량품이고, 파열된
캔은 즉시 폐기 대상이다. 이걸 분류하는 시나리오를 만들려면 정책이 **결함 자체를**
보고 판단해야 하는데, 불량품만 생김새가 통째로 다르면 라벨 그림만 외워도 맞힐 수
있다. 그래서 치수·재질·라벨이 완전히 같고 **결함 유무만 다른** 두 캔을 같은
생성기에서 뽑는다.

    normal_can.usda   정상품 — 평평한 뚜껑, 곧은 몸통
    burst_can.usda    파열품 — 아래 세 가지가 형상으로 드러난다

여기에 더해, `tools/can_designs.py` 에 적힌 도안마다 같은 한 쌍을 더 뽑는다
(sardine_can/…_burst, fig_can/…_burst). **형상·질량·충돌은 완전히 같고 라벨
이미지만 다르다** — 종류를 늘려도 정책이 물성 차이를 새로 배울 필요가 없어야
하기 때문이다. 라벨 이미지는 tools/make_can_labels.py 가 따로 그린다.

        1. 위아래 뚜껑이 크게 부풂     — 내부 압력 (실루엣에서 바로 보이도록 크게)
        2. 옆면이 깊게 찌그러짐        — 취급 중 손상
        3. 윗뚜껑이 찢겨 뜯겨 나감     — 파열, 안쪽 내용물이 드러남

정상 캔(hope/corn_can Ø68 x 58mm)과 크기를 맞췄다. 몸통은 강철 바탕에 붉은 라벨
띠를 둘렀는데, 별도 메시가 아니라 GeomSubset 으로 면을 나눠 재질만 다르게 준 것이다.

파열품은 아래 뚜껑이 볼록해 접지면이 곡면이라 벨트 위에서 비스듬히 기운다. 버그가
아니라 부푼 캔의 실제 거동이라 그대로 뒀다 — 잡기 어려운 자세라는 점이 오히려
시나리오에 맞는다.

원점은 **바운딩박스 중심**이다. franka_env/conveyor.py 가 물체의 반높이를 재서
"벨트에 얹힌 높이"를 계산하는데 그 계산이 원점=중심을 전제한다. 위아래가 비대칭이라
생성 후 실제 z 범위를 재서 원점을 다시 맞춘다.

    python3 tools/make_cans.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from can_designs import DESIGNS          # noqa: E402  (데이터만 — PIL 을 끌어오지 않는다)

# ── 치수 [m] — 두 캔이 공유한다 ────────────────────────────────────────────
R = 0.034            # 몸통 반지름 (Ø68)
H = 0.058            # 몸통 높이
RIM_R = 0.0355       # 시밍된 테두리 (몸통보다 살짝 크다)

SEG = 72             # 원주 분할
RINGS = 12           # 몸통 높이 분할
DOME_RINGS = 4       # 뚜껑 돔 분할 (많을수록 매끈)

LABEL_LO, LABEL_HI = 0.18, 0.82     # 라벨 띠가 차지하는 몸통 높이 비율

# ── 파열품에만 적용되는 결함 ──────────────────────────────────────────────
TOP_BULGE = 0.017    # 윗뚜껑이 부푼 높이 — 팽창관의 핵심 신호라 크게
BOT_BULGE = 0.008    # 아랫뚜껑이 부푼 깊이

TEAR_C = math.radians(25)     # 찢어진 구간의 중심 방향 [rad]
TEAR_HALF = math.radians(74)  # 이 각도의 윗뚜껑이 통째로 뜯겨 나가 구멍으로 남는다

DENT_C = math.radians(205)    # 옆면 찌그러짐 — 찢어진 쪽 반대편이 눌린 것으로
DENT_SIG_T = math.radians(42)
DENT_DEPTH = 0.010
DENT_Z = 0.34
DENT_SIG_Z = 0.26


def body_radius(theta: float, t: float, *, defect: bool) -> float:
    """몸통 반지름. t 는 0(바닥)~1(위) 높이 비율."""
    r = R
    r += 0.0004 * math.cos(9 * theta)                          # 아주 옅은 캔 리브
    if not defect:
        return r
    r += 0.0022 * math.sin(math.pi * t)                        # 압력으로 배가 부름
    dt = math.atan2(math.sin(theta - DENT_C), math.cos(theta - DENT_C))
    r -= DENT_DEPTH * math.exp(-(dt / DENT_SIG_T) ** 2 - ((t - DENT_Z) / DENT_SIG_Z) ** 2)
    # 눌리며 접힌 주름 — 찌그러진 쪽에서만 뚜렷하게
    fold = math.exp(-(dt / (DENT_SIG_T * 1.8)) ** 2)
    r += 0.0016 * math.sin(5 * theta + 3.0 * t) * fold
    return r


def tear_delta(theta: float) -> float:
    return math.atan2(math.sin(theta - TEAR_C), math.cos(theta - TEAR_C))


def tear_open(d: int, i: int) -> bool:
    """윗뚜껑의 이 면이 뜯겨 나갔는지. d 는 돔 링(0=테두리), i 는 원주 분할.

    경계를 각도로 흔들어 매끈한 부채꼴이 아니라 찢긴 자국으로 보이게 한다.
    흔드는 양은 **i 에만** 의존해야 한다 — 링마다 다르면 링 사이에 붕 뜬 조각이 생긴다.
    링 방향으로는 위로 갈수록 단조롭게 넓어지기만 하므로 뚫린 영역은 항상 하나로 이어진다.
    """
    a = 2 * math.pi * (i + 0.5) / SEG
    jag = math.radians(6.0) * math.sin(6.3 * a + 1.1) + math.radians(3.0) * math.sin(11.7 * a + 0.4)
    half = TEAR_HALF * (1.0 + 0.10 * d / DOME_RINGS) + jag
    return abs(tear_delta(a)) <= half


class MeshBuilder:
    def __init__(self) -> None:
        self.pts: list[tuple[float, float, float]] = []
        self.counts: list[int] = []
        self.idx: list[int] = []
        self.groups: dict[str, list[int]] = {}
        # 면꼭짓점(faceVarying) UV. self.idx 와 길이가 같다. 라벨을 붙이지 않는
        # 면도 (0,0) 으로 채워야 한다 — 길이가 어긋나면 USD 가 통째로 거부한다.
        self.uv: list[tuple[float, float]] = []

    def add(self, p) -> int:
        self.pts.append(p)
        return len(self.pts) - 1

    def ring(self, ang, radius: float, z: float) -> list[int]:
        return [self.add((radius * math.cos(a), radius * math.sin(a), z)) for a in ang]

    def face(self, *ids, group: str | None = None, uv=None) -> None:
        if group is not None:
            self.groups.setdefault(group, []).append(len(self.counts))
        self.counts.append(len(ids))
        self.idx.extend(ids)
        self.uv.extend(uv if uv is not None else [(0.0, 0.0)] * len(ids))

    def band(self, lower: list[int], upper: list[int], group: str | None = None,
             uv_rows: tuple[float, float] | None = None) -> None:
        """두 링 사이를 사각형 띠로 잇는다.

        uv_rows 를 주면 라벨 텍스처를 감는다. (아래 v, 위 v) 이고, u 는 원주를
        한 바퀴 도는 0~1 이다.

        u 를 **감긴 인덱스가 아니라 i 로** 계산하는 것이 중요하다. 마지막 칸에서
        k 가 0 으로 돌아오므로 u 를 k 로 만들면 0.99 → 0.0 이 되어, 그 한 칸에
        라벨 전체가 거꾸로 욱여넣어진다. i+1 을 쓰면 1.0 으로 닫혀 이음매가 맞는다.
        """
        n = len(lower)
        for i in range(n):
            k = (i + 1) % n
            uv = None
            if uv_rows is not None:
                u0, u1 = i / n, (i + 1) / n
                v0, v1 = uv_rows
                uv = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
            self.face(lower[i], lower[k], upper[k], upper[i], group=group, uv=uv)

    def zrange(self):
        zs = [p[2] for p in self.pts]
        return min(zs), max(zs)

    def shift_z(self, dz: float) -> None:
        self.pts = [(x, y, z + dz) for x, y, z in self.pts]


def build_shell(*, defect: bool) -> MeshBuilder:
    """몸통 + 아래뚜껑 + 위뚜껑. 파열품이면 뚜껑이 부풀고 일부가 뜯겨 나간다."""
    m = MeshBuilder()
    ang = [2 * math.pi * i / SEG for i in range(SEG)]

    # 몸통
    grid = []
    for j in range(RINGS + 1):
        t = j / RINGS
        z = -H / 2 + H * t
        grid.append([
            m.add((
                body_radius(a, t, defect=defect) * math.cos(a),
                body_radius(a, t, defect=defect) * math.sin(a),
                z,
            ))
            for a in ang
        ])
    # 라벨 띠에 해당하는 밴드 번호. v 를 LABEL_LO~LABEL_HI 비율로 직접 계산하면
    # 밴드 경계와 어긋나 라벨 위아래가 조금씩 잘려 나간다(밴드는 이산이고 비율은
    # 연속이라 그렇다). 그래서 **밴드 인덱스로** 0~1 을 나눈다.
    lab = [j for j in range(RINGS) if LABEL_LO <= (j + 0.5) / RINGS <= LABEL_HI]
    j0, span = lab[0], len(lab)
    for j in range(RINGS):
        in_label = j in lab
        uv_rows = ((j - j0) / span, (j - j0 + 1) / span) if in_label else None
        # 몸통 맨 윗 밴드도 노란 띠에 넣는다 — 측면에서 보이는 띠 두께를
        # 두 배로 키우기 위해서다 (라벨 구간은 건드리지 않는다).
        grp = ("label" if in_label
               else ("rim_top" if j >= RINGS - 2 else None))
        m.band(grid[j], grid[j + 1], group=grp, uv_rows=uv_rows)

    # 시밍된 위·아래 테두리. 위 테두리는 노란 띠(rim_top) — 탑뷰에서 캔의
    # 위치·자세가 한눈에 읽히는 시각 표지다 (task1 공구 그립 밴드와 같은 노랑,
    # 사용자 요청). 옆면 시밍 밴드와 뚜껑의 바깥 고리(아래 _cap 의 d=0)를 함께
    # 칠해 위에서도 옆에서도 보인다. 아래 테두리는 강판 그대로.
    top_rim = m.ring(ang, RIM_R, H / 2)
    bot_rim = m.ring(ang, RIM_R, -H / 2)
    m.band(grid[RINGS], top_rim, group="rim_top")
    m.band(bot_rim, grid[0])

    _cap(m, ang, bot_rim, sign=-1, bulge=BOT_BULGE if defect else 0.0, tear=False)
    _cap(m, ang, top_rim, sign=+1, bulge=TOP_BULGE if defect else 0.0, tear=defect,
         rim_group="rim_top")

    if defect:
        # 뚫린 구멍의 안쪽 벽. 배경이 비치지 않게 막으면서, 맨 금속색을 줘서
        # 찢긴 단면이 몸통보다 밝게 드러나 보이도록 한다.
        inner = m.ring(ang, R * 0.96, H / 2 - 0.016)
        for i in range(SEG):
            k = (i + 1) % SEG
            if tear_open(0, i):
                m.face(top_rim[k], inner[k], inner[i], top_rim[i], group="torn")
    return m


def _cap(m: MeshBuilder, ang, rim: list[int], *, sign: int, bulge: float, tear: bool,
         rim_group: str | None = None) -> None:
    """뚜껑 하나. bulge>0 이면 부푼 돔, 0 이면 살짝 오목한 평평한 뚜껑.

    sign 은 +1(위) / -1(아래). tear 가 참이면 뜯겨 나간 구간의 면을 만들지 않는다.
    rim_group 을 주면 뚜껑의 **바깥 고리(d=0)** 면들을 그 그룹으로 묶는다 —
    탑뷰에서 보이는 테두리 띠다. 찢긴 구간은 면이 없으므로 띠도 함께 끊긴다.
    """
    base_z = sign * H / 2
    if bulge > 0.0:
        rings = [rim]
        for d in range(1, DOME_RINGS + 1):
            f = d / DOME_RINGS
            rings.append(m.ring(
                ang,
                RIM_R * math.cos(f * math.pi / 2) ** 0.75,
                base_z + sign * bulge * math.sin(f * math.pi / 2),
            ))
        apex = m.add((0.0, 0.0, base_z + sign * bulge))
    else:
        # 정상품 뚜껑 — 테두리에서 한 단 내려앉은 평평한 패널
        rings = [rim, m.ring(ang, R * 0.955, base_z - sign * 0.0018),
                 m.ring(ang, R * 0.90, base_z - sign * 0.0030),
                 m.ring(ang, R * 0.55, base_z - sign * 0.0026),
                 m.ring(ang, R * 0.30, base_z - sign * 0.0030)]
        apex = m.add((0.0, 0.0, base_z - sign * 0.0026))

    for d in range(len(rings) - 1):
        # 바깥 세 고리(d=0~2)를 띠로 묶는다 — 두 고리(≈4mm)로도 탑뷰에서
        # 가늘어 다시 두 배로 넓혔다 (사용자 요청, 환형 폭 ≈16mm). 뚜껑
        # 중앙(반경 55% 안쪽)은 강판 그대로라 파열 부풂·찢김 신호는 남는다.
        grp = rim_group if d <= 2 else None
        for i in range(SEG):
            k = (i + 1) % SEG
            if tear and tear_open(d, i):
                continue
            # sign 에 따라 감는 방향을 뒤집어야 법선이 바깥을 본다
            if sign > 0:
                m.face(rings[d][i], rings[d + 1][i], rings[d + 1][k], rings[d][k],
                       group=grp)
            else:
                m.face(rings[d][i], rings[d][k], rings[d + 1][k], rings[d + 1][i],
                       group=grp)
    last = len(rings) - 1
    for i in range(SEG):
        k = (i + 1) % SEG
        if tear and tear_open(last, i):
            continue
        if sign > 0:
            m.face(rings[last][i], apex, rings[last][k])
        else:
            m.face(rings[last][k], apex, rings[last][i])


def build_contents() -> MeshBuilder:
    """드러난 내용물 — 구멍 아래를 메우는 울퉁불퉁한 면. 파열품에만 있다."""
    m = MeshBuilder()
    ang = [2 * math.pi * i / SEG for i in range(SEG)]
    z = H / 2 - 0.017
    c = m.add((0.0, 0.0, z - 0.003))
    ring = [
        m.add((
            (R * 0.94 + 0.0015 * math.sin(5 * a)) * math.cos(a),
            (R * 0.94 + 0.0015 * math.sin(5 * a)) * math.sin(a),
            z + 0.0015 * math.cos(4 * a) + 0.0010 * math.sin(7 * a),
        ))
        for a in ang
    ]
    for i in range(SEG):
        m.face(ring[i], ring[(i + 1) % SEG], c)
    return m


def emit(prim: str, name: str, mb: MeshBuilder, default_mat: str,
         subsets: dict, collision: bool) -> str:
    pts = ", ".join(f"({x:.5f}, {y:.5f}, {z:.5f})" for x, y, z in mb.pts)
    counts = ", ".join(str(c) for c in mb.counts)
    idx = ", ".join(str(i) for i in mb.idx)
    xs, ys, zs = ([p[i] for p in mb.pts] for i in range(3))
    schemas = ['"MaterialBindingAPI"']
    extra = ""
    if collision:
        schemas = ['"PhysicsCollisionAPI"', '"PhysicsMeshCollisionAPI"'] + schemas
        # 찌그러진 메시라 동적 강체에서는 볼록 근사가 필요하다
        extra = '        uniform token physics:approximation = "convexHull"\n'

    sub = ""
    for gname, mat in subsets.items():
        faces = mb.groups.get(gname)
        if not faces:
            continue
        sub += f'''
        def GeomSubset "{gname}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {{
            uniform token elementType = "face"
            uniform token familyName = "materialBind"
            int[] indices = [{", ".join(str(i) for i in faces)}]
            rel material:binding = </{prim}/Looks/{mat}>
        }}
'''
    uvs = ", ".join(f"({u:.5f}, {v:.5f})" for u, v in mb.uv)
    return f'''    def Mesh "{name}" (
        prepend apiSchemas = [{", ".join(schemas)}]
    )
    {{
        int[] faceVertexCounts = [{counts}]
        int[] faceVertexIndices = [{idx}]
        point3f[] points = [{pts}]
        texCoord2f[] primvars:st = [{uvs}] (
            interpolation = "faceVarying"
        )
        float3[] extent = [({min(xs):.5f}, {min(ys):.5f}, {min(zs):.5f}), ({max(xs):.5f}, {max(ys):.5f}, {max(zs):.5f})]
        uniform token subdivisionScheme = "none"
{extra}        rel material:binding = </{prim}/Looks/{default_mat}> (
            bindMaterialAs = "weakerThanDescendants"
        )
{sub}    }}'''


def material(prim: str, name: str, rgb, metallic: float, rough: float,
             texture: str | None = None) -> str:
    """OmniPBR 재질 하나. texture 를 주면 그 이미지를 확산색으로 쓴다.

    텍스처가 있어도 diffuse_color_constant 는 남겨 둔다 — 이미지를 못 찾았을 때
    캔이 새까맣게 나오는 대신 라벨 대표색으로 보이게 하는 안전망이다. 셰이더가
    이미지를 찾으면 그쪽이 이긴다.

    colorSpace 를 sRGB 로 박아야 한다. 빼면 렌더러가 선형으로 읽어 라벨이
    형광색처럼 들뜬다.
    """
    tex = ""
    if texture:
        tex = (f'\n                asset inputs:diffuse_texture = @{texture}@ '
               f'(\n                    colorSpace = "sRGB"\n                )')
    return f'''        def Material "{name}"
        {{
            token outputs:mdl:displacement.connect = </{prim}/Looks/{name}/Shader.outputs:out>
            token outputs:mdl:surface.connect = </{prim}/Looks/{name}/Shader.outputs:out>
            token outputs:mdl:volume.connect = </{prim}/Looks/{name}/Shader.outputs:out>

            def Shader "Shader"
            {{
                uniform token info:implementationSource = "sourceAsset"
                uniform asset info:mdl:sourceAsset = @OmniPBR.mdl@
                uniform token info:mdl:sourceAsset:subIdentifier = "OmniPBR"
                color3f inputs:diffuse_color_constant = ({rgb[0]}, {rgb[1]}, {rgb[2]}){tex}
                float inputs:metallic_constant = {metallic}
                float inputs:reflection_roughness_constant = {rough}
                token outputs:out (renderType = "material")
            }}
        }}'''


# normal_can 의 파열품만 이름 규칙에서 벗어난다 (normal_can_burst 가 아니라
# burst_can). 씬·태스크 정의가 그 이름을 쓰고 있어 바꾸면 다 같이 고쳐야
# 하므로, 생성기 쪽에서 이 하나만 예외로 매핑한다.
BURST_NAME = {"normal_can": "burst_can"}

# 파열품 공통 설명 — 도안별 설명(_designed_doc)에 덧붙는다.
BURST_NOTE = """
    식품 물류에서 팽창관(swollen can)은 내용물이 부패해 가스가 찬 불량품이고
    파열된 캔은 폐기 대상이다. 정상품과 치수·재질·라벨이 같고 아래 세 가지만
    다르다.
      1. 위아래 뚜껑이 크게 부풂     — 내부 압력
      2. 옆면 깊은 찌그러짐          — 취급 중 손상
      3. 윗뚜껑이 찢겨 뜯겨 나감     — 파열, 내용물 노출

    아래 뚜껑이 볼록해 접지면이 곡면이라 벨트 위에서 비스듬히 기운다. 버그가
    아니라 부푼 캔의 실제 거동이라 그대로 뒀다."""


def build_can(prim: str, doc: str, *, defect: bool, design=None) -> tuple[str, float]:
    """캔 하나. design 이 없으면 단색 붉은 라벨(대조군 normal_can/burst_can)이다."""
    shell = build_shell(defect=defect)
    parts = [shell]
    if defect:
        parts.append(build_contents())

    lo = min(p.zrange()[0] for p in parts)
    hi = max(p.zrange()[1] for p in parts)
    for p in parts:
        p.shift_z(-(lo + hi) / 2)

    if design is None:
        # 단색 대조군(normal_can/burst_can). 예전에는 붉은색이었는데 2026-08-14
        # 사용자 지시로 **전 캔 공통 마린 블루**가 됐다 — 색이 갈리면 평가에서
        # 정책이 형상 대신 색으로 정상·파열을 가를 여지가 생긴다.
        # (14, 59, 92)/255 — can_designs.py 의 field_rgb 와 같은 값이다.
        label_rgb, label_tex = (0.055, 0.231, 0.361), None
    else:
        label_rgb = tuple(round(c / 255.0, 3) for c in design.field_rgb)
        label_tex = f"./textures/{design.texture_name}"
    mats = "\n\n".join([
        material(prim, "Steel", (0.70, 0.71, 0.73), 0.88, 0.30),      # 캔 강판
        material(prim, "CanLabel", label_rgb, 0.05, 0.52, label_tex),  # 라벨 띠
        material(prim, "TornEdge", (0.80, 0.81, 0.83), 0.92, 0.22),   # 찢긴 금속 단면
        material(prim, "Contents", (0.40, 0.26, 0.12), 0.0, 0.88),    # 드러난 내용물
        # 위 테두리 띠 — 크로마키 그린·무광 (시각 표지, 사용자 요청으로
        # 노랑에서 변경: 장면에 초록 계열이 없어 가장 잘 튄다)
        material(prim, "RimBand", (0.0, 0.9, 0.2), 0.0, 0.45),
    ])
    meshes = [emit(prim, "shell", shell, "Steel",
                   {"label": "CanLabel", "torn": "TornEdge",
                    "rim_top": "RimBand"}, collision=True)]
    if defect:
        meshes.append(emit(prim, "contents", parts[1], "Contents", {}, collision=False))

    return f'''#usda 1.0
(
    doc = """{doc}

    이 파일은 tools/make_cans.py 가 생성한다. 형상을 바꾸려면 USDA 를 직접 고치지
    말고 생성기를 고쳐서 두 캔을 함께 다시 만들 것 — 짝이 어긋나면 대조군의
    의미가 없어진다.

    원점은 바운딩박스 중심이다. franka_env/conveyor.py 가 물체의 반높이를 재서
    "벨트에 얹힌 높이"를 계산하는데 그 계산이 원점=중심을 전제한다.
    """
    defaultPrim = "{prim}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{prim}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{{
    float physics:mass = 0.4

    def Scope "Looks"
    {{
{mats}
    }}

{chr(10).join(meshes)}
}}
''', hi - lo


def _designed_doc(design, *, defect: bool) -> str:
    what = "파열품" if defect else "정상품"
    burst = BURST_NAME.get(design.name, design.burst_name)
    pair = burst if not defect else design.name
    return f"""{design.brand} {design.product} — {what}.

    **모든 캔이 치수·질량·충돌 형상이 같고** 라벨 도안만 다르다
    (tools/can_designs.py). 정책이 캔 종류에 따라 다르게 잡을 이유가 없어야
    하므로, 종류를 늘릴 때 물성은 건드리지 않는다. 색도 전부 같은 파랑이다.

    짝은 {pair} 다. 둘은 같은 라벨 이미지를 쓴다 — 파열품만 라벨이 다르면
    정책이 결함이 아니라 그림을 외워서 골라내게 된다.{BURST_NOTE if defect else ""}"""


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    here = root / "env/asset/objects/cans"
    here.mkdir(parents=True, exist_ok=True)

    jobs = []
    for d in DESIGNS:
        jobs.append((d.name, _designed_doc(d, defect=False), False, d))
        jobs.append((BURST_NAME.get(d.name, d.burst_name),
                     _designed_doc(d, defect=True), True, d))

    for prim, doc, defect, design in jobs:
        text, height = build_can(prim, doc, defect=defect, design=design)
        out = here / f"{prim}.usda"
        out.write_text(text)
        if design is not None and not (here / "textures" / design.texture_name).exists():
            print(f"  경고: 라벨 이미지가 없습니다 — python3 tools/make_can_labels.py "
                  f"를 먼저 돌리세요 ({design.texture_name})")
        print(f"생성: {out.name:24s} 높이 {height * 1000:.1f}mm "
              f"(반높이 {height * 500:.1f}mm), 벨트(0.200) 위 안착 z = {0.200 + height / 2:.4f}")


if __name__ == "__main__":
    main()
