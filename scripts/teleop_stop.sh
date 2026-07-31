#!/usr/bin/env bash
# 텔레오퍼레이션 정지 (컨테이너는 그대로 둔다)
C=franka_robolab_sim
docker exec $C pkill -f "[r]un_teleop.py" 2>/dev/null && echo "✓ 정지" || echo "⚠ 실행 중이 아님"
