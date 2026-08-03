#!/usr/bin/env bash
# 시뮬레이션 초기화 — **프로세스를 죽이지 않는다.**
#
# 이걸 쓰면 sim_stop.sh + task3_train.sh 를 반복할 일이 없다. 씬을 다시 로드하는
# 데 2분이 걸리는데(셰이더 캐시가 있어도), 이 경로는 한 스텝이면 끝난다.
#
#   ./scripts/sim_reset.sh              에피소드만 다시 시작 (soft)
#   ./scripts/sim_reset.sh hard         + 로봇 관절을 강제로 되돌린다
#   ./scripts/sim_reset.sh full         + 화물을 전부 씬 기본 자세로, 장부도 0 으로
#
# 강도별 차이는 env/src/franka_env/config.py 의 RESET_LEVELS 주석에 있다.
set -euo pipefail

C=franka_robolab_sim
LEVEL="${1:-soft}"
PORT=8003
URL="http://127.0.0.1:$PORT"

case "$LEVEL" in
  soft|hard|full) ;;
  *) echo "사용법: $0 [soft|hard|full]" >&2; exit 2 ;;
esac

# 컨테이너 안에서 친다 — 심이 도는 자리와 같아야 호스트 포트 포워딩 설정에
# 기대지 않는다. 컨테이너에는 python3 가 없으므로 JSON 은 grep 으로 훑는다.
_telemetry() { docker exec "$C" curl -s --max-time 3 "$URL/telemetry" 2>/dev/null || true; }
_field() { grep -o "\"$1\": *[0-9]*" | grep -o '[0-9]*$' || true; }

raw=$(_telemetry)
if [ -z "$raw" ]; then
  echo "⚠ 시뮬레이션이 응답하지 않습니다 ($URL). 기동 중인지 확인하세요." >&2
  exit 1
fi
before=$(printf '%s' "$raw" | _field reset_count)
before=${before:-0}

docker exec "$C" curl -s --max-time 3 -X POST "$URL/reset?level=$LEVEL" >/dev/null

# 심 루프가 다음 스텝 첫머리에 가져간다. full 은 강체를 전부 다시 쓰므로
# 한 스텝이 길어질 수 있어 넉넉히 기다린다.
for _ in $(seq 40); do
  raw=$(_telemetry)
  now=$(printf '%s' "$raw" | _field reset_count)
  if [ -n "$now" ] && [ "$now" != "$before" ]; then
    kind=$(printf '%s' "$raw" | grep -o '"last_reset": *"[a-z]*"' | grep -o '[a-z]*"$' | tr -d '"')
    echo "✓ 초기화 완료 — ${kind:-?} (누적 ${now}회)"
    exit 0
  fi
  sleep 0.25
done

echo "⚠ 요청은 보냈지만 10초 안에 완료 신호가 오지 않았습니다." >&2
exit 1
