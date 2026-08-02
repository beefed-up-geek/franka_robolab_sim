#!/usr/bin/env bash
# 환경 기동 — env/script/<이름>.py 를 컨테이너 안에서 백그라운드로 띄운다.
#
#   ./scripts/sim_start.sh                 # 기본 환경(env_test)
#   ./scripts/sim_start.sh env_test        # 이름 지정
#   ./scripts/sim_start.sh env_test --camera head   # 추가 인자 전달
#
# 환경을 새로 만들면 env/script 에 파일만 추가하면 된다. 이 스크립트는 고칠 필요 없다.
set -euo pipefail

C=franka_robolab_sim
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME="${1:-env_test}"
[ $# -gt 0 ] && shift

[ -f "$REPO_DIR/env/script/${ENV_NAME}.py" ] \
    || { echo "✗ 환경 스크립트가 없습니다: env/script/${ENV_NAME}.py"; \
         echo "  사용 가능:"; ls "$REPO_DIR/env/script/" | sed 's/\.py$//; s/^/    /'; exit 1; }

[ "$(docker container inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ] \
    || { echo "✗ 컨테이너 꺼짐 → scripts/container_up.sh 먼저"; exit 1; }
docker exec $C pgrep -f "[e]nv/script/" >/dev/null 2>&1 && { echo "⚠ 이미 실행 중"; exit 0; }

mkdir -p "$REPO_DIR/logs"
docker exec -d $C bash -c \
    "/workspace/isaaclab/_isaac_sim/python.sh -u /workspace/franka_robolab_sim/env/script/${ENV_NAME}.py \
        --headless $* > /workspace/franka_robolab_sim/logs/sim.log 2>&1"

echo "기동 중: ${ENV_NAME}  (Isaac Sim 첫 실행은 셰이더 컴파일로 2~5분 걸립니다)"
for _ in $(seq 1 90); do
    sleep 5
    if grep -q "\[env\] 준비 완료" "$REPO_DIR/logs/sim.log" 2>/dev/null; then
        echo "✓ 준비 완료 → http://$(hostname -I | awk '{print $1}'):8003"
        exit 0
    fi
    if grep -qE "Traceback|\[env\] 종료:" "$REPO_DIR/logs/sim.log" 2>/dev/null; then
        echo "✗ 기동 실패 — 로그 마지막 20줄:"; tail -20 "$REPO_DIR/logs/sim.log"; exit 1
    fi
done
echo "⚠ 시간 초과 — 로그 확인: scripts/logs.sh"
tail -20 "$REPO_DIR/logs/sim.log" 2>/dev/null || true
