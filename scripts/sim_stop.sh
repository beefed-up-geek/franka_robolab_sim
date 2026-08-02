#!/usr/bin/env bash
# 환경 정지 (컨테이너는 그대로 둔다 — 셰이더 캐시가 남아 다음 기동이 빠르다)
C=franka_robolab_sim
docker exec $C pkill -f "[e]nv/script/" 2>/dev/null && echo "✓ 정지" || echo "⚠ 실행 중이 아님"
