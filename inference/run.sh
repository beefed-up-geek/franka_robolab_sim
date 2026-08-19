#!/usr/bin/env bash
# 학습한 VLA 로 시뮬레이션을 돌린다 — 추론 서버(호스트 GPU)와 구동 클라이언트
# (컨테이너 ROS)를 함께 띄운다.
#
#   ./inference/run.sh <task1|task2|task3> <모델경로> [--episodes 5]
#
# 시뮬레이션이 먼저 떠 있어야 한다: ./scripts/sim_start.sh <환경>
set -euo pipefail

TASK="${1:?task1|task2|task3}"; MODEL="${2:?pretrained_model 경로}"; shift 2 || true
C=franka_robolab_sim
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
B=/isaac-sim/exts/isaacsim.ros2.bridge/${ROS_DISTRO_BUNDLE:-jazzy}
PORT=8010

pkill -f "inference/policy_server.py" 2>/dev/null || true
sleep 1
HF_TOKEN="${HF_TOKEN:-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}" \
nohup "$HOME/hfenv/bin/python" "$REPO_DIR/inference/policy_server.py" \
    --model "$MODEL" --port $PORT ${ACTION_STEPS:+--action-steps $ACTION_STEPS} \
    > "$REPO_DIR/logs/policy_server.log" 2>&1 &
SRV=$!
trap "kill $SRV 2>/dev/null || true; docker exec $C pkill -f inference/run_policy.py 2>/dev/null || true" EXIT

echo "추론 서버 기동 중 (모델 로드 2~3분)…"
for _ in $(seq 1 120); do
    grep -q "준비 완료" "$REPO_DIR/logs/policy_server.log" 2>/dev/null && break
    grep -qE "Traceback|Error" "$REPO_DIR/logs/policy_server.log" 2>/dev/null && {
        echo "✗ 서버 기동 실패:"; tail -20 "$REPO_DIR/logs/policy_server.log"; exit 1; }
    sleep 3
done

TTY=$([ -t 1 ] && echo -t || true)
# $* 가 아니라 printf %q 로 넘긴다 — $* 는 인용을 벗겨서 공백 있는 인자
# (--instruction "reach for ...")가 단어별로 흩어져 argparse 가 즉사한다
# (실측: VLA_lang 조향 검증 B/C 가 무출력으로 사라졌다).
ARGS=$(printf "%q " "$@")
docker exec $TTY $C bash -c \
    "export PYTHONPATH=$B/rclpy:\$PYTHONPATH \
        LD_LIBRARY_PATH=$B/lib:\$LD_LIBRARY_PATH \
        RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
     /isaac-sim/python.sh /workspace/franka_robolab_sim/inference/run_policy.py \
        --task $TASK --server http://127.0.0.1:$PORT $ARGS"
