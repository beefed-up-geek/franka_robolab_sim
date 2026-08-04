#!/usr/bin/env bash
# task1 환경 — 공구 건네주기 (망치 7 · 무선드릴 · 가위, 작업자 핸드오버)
#
# 실험 조건을 인자로 매번 넘기면 기록이 남지 않는다. 이 파일이 곧 기록이다.
#   ./scripts/task1.sh                # 이 조건 그대로
set -euo pipefail

echo "[task1] 공구 3종 · 작업자 핸드오버 · 주의 테이프 구역"

exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/sim_start.sh" \
    task1_tool_handover \
    --grip-force 25 \
    "$@"
