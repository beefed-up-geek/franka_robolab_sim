#!/usr/bin/env bash
# task3 학습 환경 — 정상 통조림만 흐른다.
#
# 결함 없는 물건만 나오므로, 여기서 모은 시연으로 학습한 정책은
# 파열품을 본 적이 없다. 평가는 task3_test.sh 로 한다.
#
# 실험 조건을 인자로 매번 넘기면 어떤 조건으로 돌렸는지 기록이 남지 않는다.
# 이 파일이 곧 그 기록이다 — 조건을 바꾸려면 아래 값을 고치고 커밋한다.
#
#   ./scripts/task3_train.sh              # 이 조건 그대로
#   ./scripts/task3_train.sh --belt-speed 2  # 한 번만 다르게 (뒤에 준 인자가 이긴다)
# 조건:
#   --belt-speed 1.5  벨트 속도 [m/분]
#   --spacing 0.14    화물 간격 [m] — 지름 70mm 캔 사이에 150mm 쯤 빈다
#   --defect-ratio 0  이 씬에는 불량품이 없다. 나중에 섞이더라도 학습용은 0 으로 막는다
#   --grip-force 25     그리퍼 관절 힘 상한 [Nm]. USD 기본은 링키지가 5Nm 인데
#                       그것으로는 0.35~0.5kg 통조림을 한 번도 못 집었다(실측 0/6).
#                       25 에서 4/6. 더 올리면 폐루프 링키지가 폭주해 그리퍼가
#                       분해된다(IsaacSim #494 의 알려진 미해결 문제).
set -euo pipefail

echo "[task3-train] 정상품 4종 · 벨트 1.5 m/분 · 간격 0.14m"

exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/sim_start.sh" \
    task3_train_pick_and_place_can \
    --belt-speed 1.5 \
    --spacing 0.14   \
    --defect-ratio 0 \
    --belt-jitter 0.15 \
    --grip-force 25  \
    "$@"
