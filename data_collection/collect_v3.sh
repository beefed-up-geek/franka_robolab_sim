#!/usr/bin/env bash
# v3 수집 오케스트레이터 — 세 태스크를 200개씩 순차 수집한다.
#   nohup ./data_collection/collect_v3.sh > logs/collect_v3.log 2>&1 &
#
# 운영 사고에서 배운 것들이 코드로 들어가 있다:
#  - flock 잠금: 오케스트레이터가 2개 돌면 서로의 심·수집기를 죽인다 (실측)
#  - stop_sim: TERM 뒤 -9 백스톱, 그래도 안 죽으면 즉시 중단 — 이전 심이
#    살아 있으면 다음 수집기가 엉뚱한 씬에 붙어 데이터가 오염된다 (실측)
#  - sim_start 후 10s 대기: 부팅 직후 ROS 발견 지연으로 수집기가 헛되이
#    포기하는 것을 막는다 (수집기 자체 대기도 90s)
#  - task3 의 수집 진입점은 collect.sh 가 아니라 run.sh 다
set -uo pipefail
LOCK=/tmp/collect_v3.lock
exec 9>"$LOCK"
flock -n 9 || { echo "✗ 이미 다른 수집 오케스트레이터가 실행 중"; exit 1; }
cd "$(dirname "${BASH_SOURCE[0]}")/.."
C=franka_robolab_sim

stop_sim () {
    docker exec $C bash -c "pkill -f data_collection/ 2>/dev/null; pkill -f 'env/script/run.py' 2>/dev/null; true" || true
    for _ in $(seq 1 24); do
        docker exec $C pgrep -f "run.py .*--headless" >/dev/null 2>&1 || break
        sleep 5
    done
    if docker exec $C pgrep -f "run.py .*--headless" >/dev/null 2>&1; then
        echo "⚠ TERM 미사망 — 강제 종료"
        docker exec $C bash -c "pkill -9 -f 'env/script/run.py'; true" || true
        sleep 10
        docker exec $C pgrep -f "run.py .*--headless" >/dev/null 2>&1 && { echo "✗ 심이 죽지 않음 — 중단"; exit 2; }
    fi
    docker exec $C bash -c "rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing*" || true
    sleep 3
}

run_one () {   # run_one <환경> <수집명령...>
    local ENVN="$1"; shift
    stop_sim
    ./scripts/sim_start.sh "$ENVN" || { echo "✗ $ENVN 기동 실패"; return 1; }
    sleep 10
    "$@"
}

SEED="${SEED:-3126}"

echo "=============== task2 (200) ==============="
run_one task2_train ./data_collection/task2/collect.sh --episodes 200 --seed $SEED \
    --out /workspace/franka_robolab_sim/_data/task2/v3
echo "task2 종료 $?"

echo "=============== task1 (100/공구=200) ==============="
run_one task1 ./data_collection/task1/collect.sh --per-tool 100 --seed $SEED \
    --out /workspace/franka_robolab_sim/_data/task1/v3
echo "task1 종료 $?"

echo "=============== task3 (200) ==============="
run_one task3_train ./data_collection/task3/run.sh --episodes 200 --seed $SEED \
    --out /workspace/franka_robolab_sim/_data/task3/v3
echo "task3 종료 $?"

echo "=============== v3 수집 완료 ==============="
for t in task1 task2 task3; do
    echo "$t: $(ls _data/$t/v3/data/chunk-000/*.parquet 2>/dev/null | wc -l) 에피소드"
done
