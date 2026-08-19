#!/usr/bin/env bash
# task3 v9 — a4 학습이 끝나는 대로 체크포인트를 가져와 추론 평가까지 이어서 한다.
#
#   delta 학습 끝 → 배포 → train 환경 10회 평가
#   abs   학습 끝 → 배포 → train 환경 10회 평가 → test 환경 10회 평가(둘 다)
#
# a4 의 학습은 train_task3_v9.sh 가 delta → abs 순차로 돌린다. 이 스크립트는
# 각 체크포인트가 완성되는 대로 하나씩 처리하므로, delta 평가가 abs 학습과
# 겹쳐 돌아간다 (평가는 gty GPU, 학습은 a4 GPU 라 서로 간섭하지 않는다).
set -uo pipefail
REPO=~/franka_robolab_sim
EP="${EP:-10}"
STEP="${STEP:-025000}"
# 결과는 **지우지 않고 이어 쓴다.** 이 스크립트는 중간에 고쳐 다시 띄우는 일이
# 잦은데(전송 검증·토큰 누락으로 두 번 그랬다), 매번 비우면 이미 끝낸 평가까지
# 날아간다. 대신 아래 already_done 이 끝난 조합을 건너뛴다.
RESULT=~/eval_task3_v9_result.txt
touch "$RESULT"
echo "===== 실행 $(date '+%m-%d %H:%M') =====" >> "$RESULT"

already_done() {   # $1=KEY $2=ENV — 성공한 결과가 이미 있으면 참
    grep -A3 -- "── $1 @ $2 ──" "$RESULT" 2>/dev/null | grep -q "결과:"
}

# 체크포인트가 "다 써졌다" 고 보려면 이 파일들이 **전부** 있어야 한다.
# model.safetensors 만 보면 안 된다 — 학습이 가중치(12GB)를 먼저 쓰고 전후처리기
# json 을 나중에 쓰는데, 그 사이에 전송하면 2개짜리 반쪽 모델이 배포되고
# 추론 서버가 ProcessorMigrationError 로 죽는다 (실측 2회).
REQUIRED="config.json model.safetensors policy_preprocessor.json \
policy_postprocessor.json train_config.json"

have_all() {    # $1=목록(줄바꿈 구분) — REQUIRED 를 모두 포함하는가
    local list="$1" f
    for f in $REQUIRED; do
        echo "$list" | grep -qx -- "$f" || return 1
    done
    return 0
}

wait_ckpt() {   # $1=KEY — 체크포인트가 다 써질 때까지 기다린다
    # 표준출력은 **경로 하나만** 내보낸다 (호출부가 $( ) 로 받는다).
    # 진행 메시지를 여기에 섞으면 그대로 경로 문자열에 들어가 rsync 가 깨진다.
    local KEY="$1" SRC="" LIST=""
    echo "[eval] $KEY 학습 완료 대기…" >&2
    while true; do
        SRC=$(ssh a4 "ls -d /mnt/nas/gty/ICRA/train/out/*_${KEY}_groot-n1.7 2>/dev/null" | tail -1)
        if [ -n "$SRC" ]; then
            LIST=$(ssh a4 "ls ${SRC}/checkpoints/${STEP}/pretrained_model/ 2>/dev/null")
            if have_all "$LIST"; then
                # 필수 파일이 다 있고 크기까지 안정되면 기록이 끝난 것이다
                local A B
                A=$(ssh a4 "du -sb ${SRC}/checkpoints/${STEP}/pretrained_model 2>/dev/null | cut -f1")
                sleep 60
                B=$(ssh a4 "du -sb ${SRC}/checkpoints/${STEP}/pretrained_model 2>/dev/null | cut -f1")
                [ -n "$A" ] && [ "$A" = "$B" ] && { echo "$SRC"; return 0; }
            fi
        fi
        sleep 300
    done
}

deploy() {      # $1=KEY $2=SRC
    # 전송은 **원격 파일 목록과 대조**해서 끝났는지 본다. model.safetensors 만
    # 보면 안 된다 — 실측에서 rsync 가 12GB 가중치까지만 보내고 끊겨(gty 회선이
    # 몇 분씩 끊긴다) 전처리기 json 4개가 빠졌는데, 파일 하나만 검사하던 판정이
    # 그걸 통과시켜 추론 서버가 ProcessorMigrationError 로 죽었다.
    local KEY="$1" SRC="$2" DST=~/_model/"$1"/pretrained_model
    local REMOTE="a4:${SRC}/checkpoints/${STEP}/pretrained_model/"
    mkdir -p "$DST"
    local WANT
    WANT=$(ssh a4 "ls ${SRC}/checkpoints/${STEP}/pretrained_model/" 2>/dev/null | sort)
    [ -n "$WANT" ] || { echo "✗ $KEY 원격 목록을 못 읽었다"; return 1; }
    # 원격 목록 자체가 반쪽일 수 있다 (가중치만 쓰이고 json 은 아직인 순간).
    # 그걸 기준으로 비교하면 반쪽끼리 일치해서 통과한다 — 실측으로 두 번 당했다.
    have_all "$WANT" || { echo "✗ $KEY 원격 체크포인트가 아직 미완성"; return 1; }
    local N_WANT=$(echo "$WANT" | wc -l)

    for try in 1 2 3 4 5; do
        echo "[eval] $KEY 체크포인트 전송 (시도 $try)…"
        rsync -rlD --no-t --partial --timeout=120 "$REMOTE" "$DST/" 2>&1 | tail -1
        local HAVE=$(ls "$DST" 2>/dev/null | sort)
        if [ "$HAVE" = "$WANT" ] && have_all "$HAVE"; then
            # 가중치 크기까지 원격과 같아야 한다 (--partial 잔여분 방지)
            local RS LS
            RS=$(ssh a4 "stat -c%s ${SRC}/checkpoints/${STEP}/pretrained_model/model.safetensors" 2>/dev/null)
            LS=$(stat -c%s "$DST/model.safetensors" 2>/dev/null)
            if [ -n "$RS" ] && [ "$RS" = "$LS" ]; then
                echo "[eval] $KEY 배포 완료 — 파일 ${N_WANT}개 ($(du -sh $DST | cut -f1))"
                return 0
            fi
            echo "[eval] $KEY 가중치 크기 불일치 ($LS vs $RS) — 다시"
        else
            echo "[eval] $KEY 파일 누락 — $(echo "$HAVE" | wc -l)/${N_WANT}"
        fi
        sleep 30
    done
    echo "✗ $KEY 전송 실패 (5회)"
    return 1
}

sim_env() {     # $1=task3_train|task3_test — 필요할 때만 갈아 띄운다
    local WANT="$1"
    local CUR=$(docker exec franka_robolab_sim pgrep -af "env/script/run.py" 2>/dev/null \
                | grep -oE "task3_(train|test)" | head -1)
    [ "$CUR" = "$WANT" ] && return 0
    cd "$REPO" && ./scripts/sim_stop.sh >/dev/null 2>&1
    sleep 3
    # 로그를 **직접 비운다.** 안 비우면 아래 대기가 이전 기동의 "준비 완료" 를
    # 보고 즉시 통과한다 (sim_start.sh 의 리다이렉트 절단은 컨테이너 안에서
    # 비동기로 일어나 경합이 있다). 실측: test 환경 평가가 이 경합 때문에
    # "토픽이 오지 않습니다" 로 두 번 날아갔다.
    : > "$REPO/logs/sim.log" 2>/dev/null
    ./scripts/sim_start.sh "$WANT" >/dev/null 2>&1

    # 준비 판정은 로그가 아니라 **텔레메트리**로 한다 — 실제로 EEF 값이 나오면
    # ROS 토픽도 발행 중이라는 뜻이라, 클라이언트가 붙을 수 있는 진짜 신호다.
    local i
    for i in $(seq 1 150); do
        curl -s --max-time 3 http://127.0.0.1:8003/telemetry 2>/dev/null \
            | grep -q '"ee_x"' && break
        sleep 5
    done
    sleep 5     # 토픽 발행이 자리잡을 여유
    echo "[eval] 시뮬레이션 → $WANT (준비 확인 ${i}회차)"
}

run_eval() {    # $1=KEY $2=MODE $3=ENV
    local KEY="$1" MODE="$2" ENV="$3"
    if already_done "$KEY" "$ENV"; then
        echo "[eval] $KEY @ $ENV 이미 완료 — 건너뜀"
        return 0
    fi
    sim_env "$ENV"
    echo "[eval] $KEY @ $ENV — ${EP}회"
    local RAW OUT
    # 전체 출력을 파일로 남긴다 — [vla] 줄만 걸러 받으면 서버가 죽었을 때
    # 아무것도 안 보여 원인을 못 찾는다 (실측: 전처리기 누락으로 서버가 죽었는데
    # 결과가 그냥 빈칸으로 남았다).
    RAW=~/eval_${KEY}_${ENV}.out
    (cd "$REPO" && timeout 3000 ./inference/run.sh task3 ~/_model/"$KEY"/pretrained_model \
          --episodes "$EP" --action-mode "$MODE") > "$RAW" 2>&1
    OUT=$(grep -E "^\[vla\]" "$RAW")
    if [ -z "$OUT" ]; then
        echo "✗ $KEY @ $ENV 추론 실패 — 마지막 출력:"
        tail -5 "$RAW"
        { echo "── $KEY @ $ENV ── 실패"; tail -3 "$RAW"; echo; } >> "$RESULT"
        return 1
    fi
    echo "$OUT" | tail -12
    { echo "── $KEY @ $ENV ──"; echo "$OUT" | grep -E "ep[0-9]+:|결과:"; echo; } >> "$RESULT"
}

diagnose() {    # $1=KEY — 개루프 + 다봉성 진단 (폐루프가 나쁠 때 원인을 가른다)
    # 오류를 삼키지 않는다 — 조용히 빈 결과를 남기면 진단이 없는 것과 같다.
    local KEY="$1" M=~/_model/"$1"/pretrained_model D=~/_lerobot/"$1"
    [ -d "$D" ] || { echo "[eval] $KEY 데이터셋 없음 — 진단 건너뜀"; return 0; }
    # GR00T 토크나이저가 게이트 저장소(nvidia/Cosmos-Reason2-2B)를 받는다 —
    # 토큰이 없으면 401 GatedRepoError 로 죽는다. inference/run.sh 는 이걸
    # 주입하지만 파이썬을 직접 부르는 여기서는 따로 넣어야 한다.
    export HF_TOKEN="${HF_TOKEN:-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}"
    echo "[eval] $KEY 개루프·다봉성 진단"
    { echo "── $KEY 개루프 ──"
      ~/hfenv/bin/python ~/openloop_v3.py "$M" "$D" 3 2>&1 \
          | grep -E "^ep|카메라|Error|error|Traceback" | tail -6
      echo "── $KEY 다봉성 ──"
      ~/hfenv/bin/python ~/modecheck_v8.py "$M" "$D" 3 2>&1 \
          | grep -E "^ep|판정|분산이|Error|error|Traceback" | tail -6
      echo; } | tee -a "$RESULT"
}

for MODE in delta abs; do
    KEY=task3_${MODE}_v9
    SRC=$(wait_ckpt "$KEY") || continue
    deploy "$KEY" "$SRC" || continue
    diagnose "$KEY"
    run_eval "$KEY" "$MODE" task3_train
done

# test 환경(정상 캔만 담기)은 두 모델을 이어서 본다 — 시뮬 재기동이 한 번뿐이다.
for MODE in delta abs; do
    KEY=task3_${MODE}_v9
    [ -f ~/_model/"$KEY"/pretrained_model/model.safetensors ] || continue
    run_eval "$KEY" "$MODE" task3_test
done

echo "[eval] 전체 완료 $(date +%H:%M)"
cat "$RESULT"
