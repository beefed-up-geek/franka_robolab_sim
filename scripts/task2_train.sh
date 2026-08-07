#!/usr/bin/env bash
# task2 환경 — 발전기 플러그를 배터리에 연결 (구축 1단계: 기물 배치)
#
#   ./scripts/task2.sh                # 이 조건 그대로
set -euo pipefail

echo "[task2-train] SAM3D 배터리·발전기 배치 · 플러그 연결 태스크(구축 중)"

exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/sim_start.sh" \
    task2_train_charging \
    --grip-force 25 \
    "$@"
