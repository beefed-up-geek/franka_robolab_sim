#!/usr/bin/env bash
# 그리퍼 힘 상한 x 화물 질량 조합 실험.
#
# 각 조합마다 시뮬레이션을 새로 띄우고 정해진 횟수만큼 집기를 시도한 뒤,
# 성공 / 폭주 / 낙하를 센다. 조합당 약 8분.
#
# 주의: grep -c 는 매치가 없으면 "0" 을 찍고 종료코드 1 을 낸다. `|| echo 0` 을
# 붙이면 "0\n0" 이 되어 산술식이 깨진다 — 그래서 붙이지 않는다.
set -uo pipefail
cd ~/franka_robolab_sim

ATTEMPTS=6
SEED=41
COMBOS=(
  "5.0   0.0"    # RoboLab 원본 (힘 5Nm, 질량 0.35~0.5kg)
  "12.0  0.0"    # 힘만 소폭
  "25.0  0.0"    # 현재 설정
  "5.0   0.15"   # 원본 힘 + 가벼운 캔
  "12.0  0.15"   # 중간 + 가벼운 캔
  "25.0  0.30"   # 높은 힘 + 중간 질량
)

count() { grep -c "$1" <<<"$2" || true; }

printf "%-6s %-6s %5s %5s %5s %8s %8s %5s\n" \
       force mass 성공 폭주 낙하 파지실패 운반놓침 기타
for c in "${COMBOS[@]}"; do
  set -- $c; F=$1; M=$2
  docker exec franka_robolab_sim bash -lc "pkill -9 -f collect.py 2>/dev/null; true"
  ./scripts/sim_stop.sh >/dev/null 2>&1
  for i in $(seq 1 40); do
    docker exec franka_robolab_sim pgrep -f "env/script/" >/dev/null 2>&1 || break
    sleep 2
  done
  ./scripts/task3_train.sh --grip-force "$F" --can-mass "$M" >/dev/null 2>&1
  if ! grep -q "\[env\] 준비 완료" logs/sim.log 2>/dev/null; then
    printf "%-6s %-6s 기동 실패\n" "$F" "$M"; continue
  fi
  before=$(grep -c "^\[recycle\].*벨트밖" logs/sim.log || true)
  out=$(timeout 420 ./data_collection/task3/run.sh --episodes "$ATTEMPTS" \
          --max-attempts "$ATTEMPTS" --out "/tmp/sweep_${F}_${M}" --seed "$SEED" 2>&1)
  after=$(grep -c "^\[recycle\].*벨트밖" logs/sim.log || true)
  ok=$(count "저장 —" "$out")
  boom=$(count "폭주" "$out")
  gf=$(count "파지 실패" "$out")
  cl=$(count "운반 중 놓침" "$out")
  fail=$(count "실패 —" "$out")
  printf "%-6s %-6s %5s %5s %5s %8s %8s %5s\n" \
         "$F" "$M" "$ok" "$boom" "$((after-before))" "$gf" "$cl" \
         "$((fail-boom-gf-cl))"
done
