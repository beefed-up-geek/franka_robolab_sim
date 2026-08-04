#!/usr/bin/env bash
# task3 평가 환경 — 팽창·파열된 불량품이 섞여 흐른다.
#
# train 과 설비는 완전히 같고(씬이 _can_workcell.usda 를 공유한다)
# 흐르는 물건만 다르다. 학습 때 못 본 조건이 나온다.
#
# 실험 조건을 인자로 매번 넘기면 어떤 조건으로 돌렸는지 기록이 남지 않는다.
# 이 파일이 곧 그 기록이다 — 조건을 바꾸려면 아래 값을 고치고 커밋한다.
#
#   ./scripts/task3_test.sh              # 이 조건 그대로
#   ./scripts/task3_test.sh --belt-speed 2  # 한 번만 다르게 (뒤에 준 인자가 이긴다)
# 조건:
#   --belt-speed 1.5  벨트 속도 [m/분]
#   --spacing 0.14      화물 간격 [m]
#   --defect-ratio 0.2  투입 화물 중 불량품 비율. 장기 평균으로만 맞는다
#   --grip-force 25     그리퍼 관절 힘 상한 [Nm]. USD 기본은 링키지가 5Nm 인데
#                       그것으로는 0.35~0.5kg 통조림을 한 번도 못 집었다(실측 0/6).
#                       25 에서 4/6. 더 올리면 폐루프 링키지가 폭주해 그리퍼가
#                       분해된다(IsaacSim #494 의 알려진 미해결 문제).
set -euo pipefail

echo "[task3-test] 정상 4종 + 불량 4종 · 불량 20% · 벨트 1.5 m/분 · 간격 0.14m"

exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/sim_start.sh" \
    task3_test_pick_and_place_can \
    --belt-speed 1.5 \
    --spacing 0.14     \
    --defect-ratio 0.2 \
    --belt-jitter 0.15 \
    --grip-force 25  \
    "$@"
