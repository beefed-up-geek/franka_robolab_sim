#!/usr/bin/env python3
"""task3 학습/평가 환경 두 벌을 만든다.

정상품만 흐르는 train 과, 팽창·파열된 불량품이 섞여 흐르는 test 로 나눈다.
작업대·컨베이어·조명 같은 고정 설비는 두 씬이 `_can_workcell.usda` 를 서브레이어로
공유한다. 그 파일은 여기서 만들지 않는다 — 직접 관리하는 소스다.
복사해 두면 한쪽만 고쳐져 train/test 가 조용히 달라지고, 그러면 평가 결과가
환경 차이 때문인지 정책 때문인지 구분할 수 없게 된다.
"""
import pathlib

R = pathlib.Path.home() / "franka_robolab_sim"
SCENES = R / "env/asset/scenes"

# 이름: (payload 경로, 벨트 위 안착 z)
#
# 여기 없는 캔은 씬에 들어가지 않는다. 파일이 남아 있어도 마찬가지다 — 목록이
# 곧 편성이다.
CANS = {
    "corn_can":          ("../objects/cans/corn_can.usda", 0.2290),
    "normal_can":        ("../objects/cans/normal_can.usda", 0.2290),
    "sardine_can":       ("../objects/cans/sardine_can.usda", 0.2290),
    "fig_can":           ("../objects/cans/fig_can.usda", 0.2290),
    "corn_can_burst":    ("../objects/cans/corn_can_burst.usda", 0.2415),
    "burst_can":         ("../objects/cans/burst_can.usda", 0.2415),
    "sardine_can_burst": ("../objects/cans/sardine_can_burst.usda", 0.2415),
    "fig_can_burst":     ("../objects/cans/fig_can_burst.usda", 0.2415),
}
# 정상 4종은 형상·질량이 완전히 같고 라벨만 다르다. 정책이 "종류가 늘었다" 를
# 물성 변화로 배우지 않게 하려는 것이다 (tools/can_designs.py, cans/README.md).
# corn_can 도 원래는 외부 HOPE 에셋(hope/corn_can.usd)이었는데, 물성이 달라
# 파지 때 그리퍼 폭주를 일으켜 생성 캔으로 교체했다 (2026-08-10).
NORMAL = ["corn_can", "normal_can", "sardine_can", "fig_can"]
BURST = ["corn_can_burst", "burst_can", "sardine_can_burst", "fig_can_burst"]

BELT_Y = (-0.30, -0.06, 0.18)   # 벨트에 처음부터 올려 두는 자리. 간격 0.24m.
BELT_X = 0.52


def can_block(name: str, y: float, z: float, indent: str = "    ") -> str:
    payload, _ = CANS[name][0], None
    return (f'{indent}def "{name}" (\n'
            f'{indent}    prepend payload = @{payload}@\n'
            f'{indent})\n'
            f'{indent}{{\n'
            f'{indent}    quatf xformOp:orient = (1, 0, 0, 0)\n'
            f'{indent}    float3 xformOp:scale = (1, 1, 1)\n'
            f'{indent}    double3 xformOp:translate = ({BELT_X}, {round(y, 3)}, {z})\n'
            f'{indent}    uniform token[] xformOpOrder = '
            f'["xformOp:translate", "xformOp:orient", "xformOp:scale"]\n'
            f'{indent}}}\n')


def scene(name: str, doc: str, belt: list, queue: list) -> str:
    body = "".join(can_block(n, BELT_Y[i], CANS[n][1]) for i, n in enumerate(belt))
    body += "\n" + "".join(
        can_block(n, -0.36 + 0.12 * i, -0.30) for i, n in enumerate(queue))
    return f'''#usda 1.0
(
    doc = """{doc}"""
    subLayers = [
        @./_can_workcell.usda@
    ]
    defaultPrim = "world"
    endTimeCode = 0
    kilogramsPerUnit = 1
    metersPerUnit = 1
    startTimeCode = -1
    upAxis = "Z"
)

over "world"
{{
    # -- 벨트에 처음부터 올려 두는 캔 (간격 0.24m) -------------------------
{body}}}
'''


TRAIN_DOC = """task3 학습 씬 — 정상품만 흐른다.

    통조림 4종이 벨트를 타고 흘러온다. 벨트에는 3개만 올리고 나머지는 상판 아래에서
    대기로 시작한다 — 출구를 지난 것이 대기열로 들어가고 입구가 비면 하나씩
    투입되므로, 결과적으로 4종이 돌아가며 흐른다(franka_env/conveyor.py 의 recycle).

    그중 normal_can · sardine_can · fig_can 은 **치수·질량·충돌 형상이 완전히 같고
    라벨 도안만 다르다.** 잡는 법은 그대로 두고 보이는 것만 바꾸려는 것이다 —
    라벨이 달라도 같은 동작으로 집힌다는 것을 시연으로 보여 주면, 정책이 캔 종류에
    동작을 결부시키지 않는다.

    캔 원점은 **중심**이라 벨트에 얹으려면 반높이만큼 올려야 하고, 아래 translate 의
    z 가 그렇게 계산된 값이다.

    고정 설비는 _can_workcell.usda 를 서브레이어로 공유한다."""

TEST_DOC = """task3 평가 씬 — 팽창·파열된 불량품이 섞여 흐른다.

    train 과 흐르는 물건만 다르고 설비는 완전히 같다(_can_workcell.usda 공유).
    정상품 4종에 그 짝인 파열품 4종을 더해 8종이 돌아간다.

    짝을 맞춘 이유는 정책이 **결함 자체를** 보게 하기 위해서다. 불량품만 라벨이
    다르면 그림만 외워도 골라낼 수 있다. 파열품은 원본 메시를 변형해 만든 것이라
    텍스처·UV 가 정상품과 같고, 부푼 뚜껑·찌그러진 옆면·뜯긴 구멍만 다르다
    (env/asset/objects/cans/README.md 참고).

    파열품은 아래 뚜껑이 볼록해 벨트 위에서 비스듬히 기운다. 잡기 까다로운 자세라
    train 에서 보지 못한 조건이 된다."""

# 벨트에는 정상품 3개만 올려 두고 나머지는 전부 대기열로 보낸다. 불량품 투입은
# --defect-ratio 가 정하므로, 시작부터 불량품을 얹어 두면 그 비율과 어긋난다.
# 대기열은 벨트 목록에서 **자동으로 유도한다** — 손으로 적으면 한쪽만 고쳤을 때
# 캔이 씬에서 통째로 빠지고, 태스크가 그 프림을 찾지 못해 기동이 실패한다.
START_BELT = ["corn_can", "normal_can", "sardine_can"]
for fname, doc, cargo in (
    ("task3_train.usda", TRAIN_DOC, NORMAL),
    ("task3_test.usda", TEST_DOC, NORMAL + BURST),
):
    assert all(n in cargo for n in START_BELT), fname
    SCENES.joinpath(fname).write_text(
        scene(fname, doc, START_BELT, [n for n in cargo if n not in START_BELT]))

for f in ("_can_workcell.usda", "task3_train.usda", "task3_test.usda"):
    t = (SCENES / f).read_text()
    assert t.count("{") == t.count("}"), (f, t.count("{"), t.count("}"))
    print(f"{f:24s} {len(t.splitlines()):4d}줄  중괄호 OK")
