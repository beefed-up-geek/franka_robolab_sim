#!/usr/bin/env bash
# 학습된 모델 전부를 각자의 환경에서 시연한다.
#
#   ./demo_all.sh [에피소드수=5] [모델 필터 정규식]
#
# 브라우저(:8003)로 보면서 돌리는 용도라 환경 전환 때마다 기동 완료를 확인한다.
# 결과는 ~/demo_all_result.txt 에 쌓인다.
set -uo pipefail
EP="${1:-5}"
FILTER="${2:-.}"
REPO=~/franka_robolab_sim
RESULT=~/demo_all_result.txt
echo "===== 시연 $(date '+%m-%d %H:%M') · 에피소드 ${EP}회 =====" >> "$RESULT"

# 태스크 · 환경 · 모델키 · 액션모드
JOBS=(
  "task3 task3_test  task3_abs_v9    abs"
  "task3 task3_test  task3_delta_v9  delta"
  "task2 task2_test  task2_abs       abs"
  "task2 task2_test  task2_delta     delta"
  "task1 task1       task1_abs       abs"
  "task1 task1       task1_delta     delta"
)

sim_env() {     # $1=환경 이름 — 필요할 때만 갈아 띄우고 준비를 확인한다
    local WANT="$1"
    local CUR
    CUR=$(docker exec franka_robolab_sim pgrep -af "env/script/run.py" 2>/dev/null \
          | grep -oE "task[123](_train|_test)?" | head -1)
    [ "$CUR" = "$WANT" ] && return 0
    echo "[demo] 환경 전환 → $WANT (기동 2~5분)"
    cd "$REPO" && ./scripts/sim_stop.sh >/dev/null 2>&1
    # **프로세스가 실제로 사라질 때까지 기다린다.** sim_start.sh 는 남아 있는
    # 프로세스를 보면 "이미 실행 중" 으로 즉시 빠져나가는데, 종료 중인 심이
    # 그 뒤 죽으면 아무것도 안 뜬 채로 진행된다 (실측: task2 시연이 통째로
    # 날아갔다).
    local w
    for w in $(seq 1 30); do
        docker exec franka_robolab_sim pgrep -f "env/script/run.py" >/dev/null 2>&1 || break
        sleep 2
    done
    sleep 2
    ./scripts/sim_start.sh "$WANT" >/dev/null 2>&1
    # 준비 판정은 로그가 아니라 텔레메트리로 — 로그에는 이전 기동의
    # "준비 완료" 가 남아 있어 즉시 통과해 버린다 (실측 2회).
    # **키가 아니라 값이 있는지**를 본다: 웹 서버는 심 루프보다 먼저 뜨고
    # 그동안 "ee_x": null 을 내보내므로, 키만 찾으면 또 일찍 통과한다.
    local i
    for i in $(seq 1 150); do
        curl -s --max-time 3 http://127.0.0.1:8003/telemetry 2>/dev/null \
            | grep -qE '"ee_x": *-?[0-9]' && break
        sleep 5
    done
    sleep 5
}

for job in "${JOBS[@]}"; do
    read -r TASK ENV KEY MODE <<< "$job"
    echo "$KEY" | grep -qE "$FILTER" || continue
    M=~/_model/"$KEY"/pretrained_model
    [ -f "$M/model.safetensors" ] || { echo "[demo] $KEY 모델 없음 — 건너뜀"; continue; }
    sim_env "$ENV"
    echo "── $KEY @ $ENV (${MODE}) ──"
    RAW=~/demo_${KEY}_${ENV}.out
    (cd "$REPO" && timeout 1800 ./inference/run.sh "$TASK" "$M" \
        --episodes "$EP" --action-mode "$MODE") > "$RAW" 2>&1
    OUT=$(grep -aE "^\[vla\]" "$RAW")
    if [ -z "$OUT" ]; then
        echo "✗ 실행 실패 — 마지막 출력:"; tail -3 "$RAW"
        { echo "── $KEY @ $ENV ── 실패"; tail -2 "$RAW"; echo; } >> "$RESULT"
        continue
    fi
    echo "$OUT" | tail -8
    { echo "── $KEY @ $ENV (${MODE}) ──"
      echo "$OUT" | grep -aE "ep[0-9]+:|결과:"; echo; } >> "$RESULT"
done

echo "[demo] 전체 완료 $(date +%H:%M)"
cat "$RESULT"
