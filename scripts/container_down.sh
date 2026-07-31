#!/usr/bin/env bash
# 컨테이너 정지 (삭제하지 않는다 — 다시 올리려면 container_up.sh)
C=franka_robolab_sim
docker stop $C >/dev/null 2>&1 && echo "✓ 정지: $C" || echo "⚠ 실행 중이 아님: $C"
