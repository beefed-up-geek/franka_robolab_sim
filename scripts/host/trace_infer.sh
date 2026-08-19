#!/usr/bin/env bash
# 추론을 돌리면서 EEF 궤적을 기록한다 — 폐루프가 실패할 때 "팔이 무엇을 했는가"
# 를 보는 진단. 성공/실패만으로는 얼어붙은 것인지, 헤맨 것인지, 잡았다 놓친
# 것인지 구분할 수 없다.
#
#   ./trace_infer.sh <KEY> <delta|abs> [에피소드=2]
set -uo pipefail
KEY="${1:?KEY}"; MODE="${2:?delta|abs}"; EP="${3:-2}"
REPO=~/franka_robolab_sim
TRACE=~/trace_${KEY}.tsv

pkill -f "inference/policy_server[.]py" 2>/dev/null
docker exec franka_robolab_sim pkill -f "run_polic[y].py" 2>/dev/null
sleep 2

# 텔레메트리 폴링 — 0.5초마다 EEF·그리퍼·통 카운터를 찍는다
( echo -e "t\tee_x\tee_y\tee_z\tgrip\tbinned\ton_belt\tqueued"
  for _ in $(seq 1 600); do
      curl -s --max-time 2 http://127.0.0.1:8003/telemetry 2>/dev/null \
        | python3 -c '
import json,sys,time
try:
    d=json.load(sys.stdin)
    print("%.1f\t%s\t%s\t%s\t%s\t%s\t%s\t%s" % (time.time()%10000,
      d.get("ee_x"), d.get("ee_y"), d.get("ee_z"), d.get("gripper"),
      d.get("binned"), d.get("on_belt"), d.get("queued")))
except Exception: pass' 2>/dev/null
      sleep 0.5
  done ) > "$TRACE" &
POLL=$!
trap "kill $POLL 2>/dev/null" EXIT

cd "$REPO" && timeout 400 ./inference/run.sh task3 ~/_model/"$KEY"/pretrained_model \
    --episodes "$EP" --action-mode "$MODE" 2>&1 | grep -E "^\[vla\]"

kill $POLL 2>/dev/null
echo "── 궤적 요약 ($(wc -l < "$TRACE") 샘플) ──"
python3 - "$TRACE" <<'PY'
import sys
rows=[l.split("\t") for l in open(sys.argv[1]) if l.strip()][1:]
pts=[]
for r in rows:
    try: pts.append((float(r[1]),float(r[2]),float(r[3]),r[4],int(r[5])))
    except Exception: pass
if not pts: print("샘플 없음"); raise SystemExit
xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; zs=[p[2] for p in pts]
print(f"x {min(xs):.3f}~{max(xs):.3f}  y {min(ys):.3f}~{max(ys):.3f}  z {min(zs):.3f}~{max(zs):.3f}")
print(f"그리퍼 CLOSED 비율 {sum(p[3]=='CLOSED' for p in pts)/len(pts):.0%}  ·  통 {pts[0][4]}→{pts[-1][4]}")
# 정지 구간: 연속 10샘플(5초) 동안 3mm 미만 이동
frozen=0; run=0
for a,b in zip(pts,pts[1:]):
    d=sum((a[i]-b[i])**2 for i in range(3))**0.5
    run = run+1 if d<0.003 else 0
    frozen=max(frozen,run)
print(f"최장 정지 {frozen*0.5:.1f}초 (3mm 미만 이동 연속)")
# 벨트 위(x 0.44~0.60) 체류 비율과 하강 도달 최저 z
onbelt=[p for p in pts if 0.44<p[0]<0.60]
print(f"벨트 상공 체류 {len(onbelt)/len(pts):.0%}  ·  벨트 위에서 최저 z {min((p[2] for p in onbelt), default=float('nan')):.3f}")
print(f"마지막 위치 ({pts[-1][0]:.3f}, {pts[-1][1]:.3f}, {pts[-1][2]:.3f}) 그리퍼 {pts[-1][3]}")
PY
