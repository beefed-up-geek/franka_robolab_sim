#!/usr/bin/env bash
# task3 v9 수집이 끝나면 delta·abs 변환 → HF 공개 업로드 → a4 전송 →
# a4 학습(train_task3_v9.sh) 착수까지 이어서 한다.
set -uo pipefail
HF=~/hfenv/bin/python
LOG=/home/gty/collect_task3_v9.log
DST=~/_lerobot

echo "[post] task3 v9 수집 종료 대기…"
while ! grep -q "파지 y 분포" "$LOG" 2>/dev/null; do sleep 120; done
echo "[post] 수집 종료 $(date +%H:%M)"

SRC=$(ls -td ~/franka_robolab_sim/_data/task3/v9_main* 2>/dev/null | head -1)
[ -n "$SRC" ] && [ -f "$SRC/meta/info.json" ] || { echo "✗ 원본 없음"; exit 1; }
N=$(ls "$SRC/data/chunk-000/"*.parquet 2>/dev/null | wc -l)
[ "$N" -ge 195 ] || { echo "✗ 에피소드 부족($N)"; exit 1; }
echo "[post] 원본 $SRC ($N 에피소드)"

declare -A REPO=(
  [task3_delta_v9]=franka_task3_conveyor_pick_delta_v9
  [task3_abs_v9]=franka_task3_conveyor_pick_abs_v9
)
for MODE in delta abs; do
  KEY=task3_${MODE}_v9
  echo "=== [$KEY] 변환 $(date +%H:%M) ==="
  $HF ~/ingest.py --src "$SRC" --dst "$DST/$KEY" --action $MODE | tail -1
  echo "=== [$KEY] 업로드 → ${REPO[$KEY]} ==="
  $HF ~/upload_one.py "$KEY" "${REPO[$KEY]}" 2>&1 | tail -1
  rsync -rlD --no-t --delete "$DST/$KEY/" "a4:/mnt/nas/gty/ICRA/train/data/${KEY}/" 2>&1 | tail -1
  echo "=== [$KEY] a4 전송 완료 ==="
done

echo "[post] a4 GPU 상태 확인"
ssh a4 "docker exec groot_task3 nvidia-smi --query-gpu=memory.used --format=csv,noheader" \
  || { echo "✗ a4 NVML 이상 — 컨테이너 재시작"; ssh a4 "docker restart groot_task3"; sleep 20; }

echo "[post] a4 학습 착수"
ssh a4 "docker exec -d groot_task3 bash -c '/workspace/train_task3_v9.sh 25000 16 > /workspace/logs/train_task3_v9_all.log 2>&1'"
echo "[post] 전체 완료 $(date +%H:%M)"
