#!/usr/bin/env python3
import pathlib, stat
R = pathlib.Path.home() / "franka_robolab_sim"

TMPL = '''#!/usr/bin/env bash
# {title}
#
# {why}
#
# 실험 조건을 인자로 매번 넘기면 어떤 조건으로 돌렸는지 기록이 남지 않는다.
# 이 파일이 곧 그 기록이다 — 조건을 바꾸려면 아래 값을 고치고 커밋한다.
#
#   ./scripts/{name}              # 이 조건 그대로
#   ./scripts/{name} --seed 7     # 한 번만 다르게 (뒤에 준 인자가 이긴다)
set -euo pipefail

echo "[{label}] {summary}"

exec "$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)/sim_start.sh" \\
    {env} \\
{args}    "$@"
'''

RUNNERS = [
    dict(
        name="task3_train.sh",
        label="task3-train",
        env="task3_train_pick_and_place_can",
        title="task3 학습 환경 — 정상 통조림만 흐른다.",
        why=("결함 없는 물건만 나오므로, 여기서 모은 시연으로 학습한 정책은\n"
             "# 파열품을 본 적이 없다. 평가는 task3_test.sh 로 한다."),
        summary="정상품 5종 · 벨트 2 m/분 · 간격 0.22m · 시드 0",
        args=[
            ("--belt-speed 2", "벨트 속도 [m/분]"),
            ("--spacing 0.22", "화물 간격 [m] — 지름 70mm 캔 사이에 150mm 쯤 빈다"),
            ("--defect-ratio 0", "이 씬에는 불량품이 없다. 나중에 섞이더라도 학습용은 0 으로 막는다"),
            ("--seed 0", "투입 순서 — 같은 시드면 같은 순서가 재현된다"),
        ],
    ),
    dict(
        name="task3_test.sh",
        label="task3-test",
        env="task3_test_pick_and_place_can",
        title="task3 평가 환경 — 팽창·파열된 불량품이 섞여 흐른다.",
        why=("train 과 설비는 완전히 같고(씬이 _can_workcell.usda 를 공유한다)\n"
             "# 흐르는 물건만 다르다. 학습 때 못 본 조건이 나온다."),
        summary="정상 5종 + 불량 5종 · 불량 20% · 벨트 2 m/분 · 간격 0.22m · 시드 0",
        args=[
            ("--belt-speed 2", "벨트 속도 [m/분]"),
            ("--spacing 0.22", "화물 간격 [m]"),
            ("--defect-ratio 0.2", "투입 화물 중 불량품 비율. 장기 평균으로만 맞는다"),
            ("--seed 0", "투입 순서 — 같은 시드면 같은 순서가 재현된다"),
        ],
    ),
]

for r in RUNNERS:
    width = max(len(a) for a, _ in r["args"])
    args = "".join(f"    {a:<{width}} \\\n" if i < len(r["args"]) - 1
                   else f"    {a:<{width}} \\\n"
                   for i, (a, _) in enumerate(r["args"]))
    # 각 인자 옆에 뜻을 주석으로 붙이면 백슬래시 이어쓰기가 깨지므로 위에 모아 둔다.
    header = "".join(f"#   {a:<{width}}  {c}\n" for a, c in r["args"])
    body = TMPL.format(name=r["name"], label=r["label"], env=r["env"],
                       title=r["title"], why=r["why"],
                       summary=r["summary"], args=args)
    body = body.replace("set -euo pipefail\n",
                        "# 조건:\n" + header + "set -euo pipefail\n", 1)
    f = R / "scripts" / r["name"]
    f.write_text(body)
    f.chmod(f.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    print(f"생성: scripts/{r['name']}")
