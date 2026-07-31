#!/usr/bin/env bash
# 텔레오퍼레이션 기동 → 브라우저에서 http://<서버주소>:8003 접속
set -euo pipefail

C=franka_robolab_sim
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ "$(docker container inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ] \
    || { echo "✗ 컨테이너 꺼짐 → scripts/container_up.sh 먼저"; exit 1; }
docker exec $C pgrep -f "[r]un_teleop.py" >/dev/null 2>&1 && { echo "⚠ 이미 실행 중"; exit 0; }

docker exec -d $C bash -c \
    '/workspace/isaaclab/_isaac_sim/python.sh -u /workspace/franka_robolab_sim/run_teleop.py --headless \
        > /workspace/franka_robolab_sim/logs/teleop.log 2>&1'

echo "기동 중... (Isaac Sim 첫 실행은 셰이더 컴파일로 2~5분 걸립니다)"
for _ in $(seq 1 60); do
    sleep 5
    if grep -q "\[teleop\] 준비 완료" "$REPO_DIR/logs/teleop.log" 2>/dev/null; then
        echo "✓ 준비 완료 → http://$(hostname -I | awk '{print $1}'):8003"
        exit 0
    fi
    if grep -qE "Traceback|Error|종료:" "$REPO_DIR/logs/teleop.log" 2>/dev/null; then
        echo "✗ 기동 실패 — 로그 마지막 20줄:"; tail -20 "$REPO_DIR/logs/teleop.log"; exit 1
    fi
done
echo "⚠ 시간 초과 — 로그 확인: scripts/logs.sh"
tail -20 "$REPO_DIR/logs/teleop.log" 2>/dev/null || true
