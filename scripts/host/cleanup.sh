#!/usr/bin/env bash
# 허깅페이스에 올린 로컬 사본을 지우고(HF 원본은 유지) v3 수집을 새로 시작한다.
set -uo pipefail
C=franka_robolab_sim
pkill -f collect_v3.sh 2>/dev/null || true
docker exec $C bash -c "pkill -f data_collection/ 2>/dev/null; true" || true
sleep 3
docker exec $C bash -c "rm -rf /workspace/franka_robolab_sim/_data/*" || true
rm -rf ~/_lerobot/task1_delta ~/_lerobot/task2_delta ~/_lerobot/task3_delta ~/_lerobot/task2_abs
echo "--- 정리 후 ---"
du -sh ~/franka_robolab_sim/_data ~/_lerobot 2>/dev/null
