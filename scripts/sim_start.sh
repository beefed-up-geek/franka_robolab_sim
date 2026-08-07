#!/usr/bin/env bash
# 환경 기동 — env/script/run.py 에 환경 이름을 넘겨 컨테이너 안에서 백그라운드로 띄운다.
#
#   ./scripts/sim_start.sh task2_train
#   ./scripts/sim_start.sh task3_test --belt-speed 2.0
#
# 사용 가능한 환경과 환경별 기본 인자(실험 조건)는 env/src/franka_env/envs.py
# 레지스트리가 정한다. 새 환경 = env/src/tasks 에 태스크 정의 1개 + 레지스트리 한 줄.
set -euo pipefail

C=franka_robolab_sim
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 레지스트리는 순수 표준 라이브러리라 호스트 python3 로 바로 읽는다
# (franka_env 패키지 __init__ 을 타지 않도록 파일을 직접 로드).
list_envs() {
    python3 - "$REPO_DIR" <<'PY'
import importlib.util, sys
spec = importlib.util.spec_from_file_location(
    "envs", sys.argv[1] + "/env/src/franka_env/envs.py")
m = importlib.util.module_from_spec(spec)
sys.modules["envs"] = m      # dataclasses 가 sys.modules 로 모듈을 찾는다
spec.loader.exec_module(m)
for n, s in m.ENVS.items():
    print(f"    {n:<12} {s.description}")
PY
}

env_exists() {
    python3 - "$REPO_DIR" "$1" <<'PY'
import importlib.util, sys
spec = importlib.util.spec_from_file_location(
    "envs", sys.argv[1] + "/env/src/franka_env/envs.py")
m = importlib.util.module_from_spec(spec)
sys.modules["envs"] = m      # dataclasses 가 sys.modules 로 모듈을 찾는다
spec.loader.exec_module(m)
sys.exit(0 if sys.argv[2] in m.ENVS else 1)
PY
}

# 기본값을 두지 않는다. 어떤 환경을 띄웠는지 모호해지면 실험 기록이 어긋난다.
ENV_NAME="${1:-}"
[ -n "$ENV_NAME" ] \
    || { echo "✗ 환경 이름이 필요합니다."; echo "  사용 가능:"; list_envs; exit 1; }
shift

env_exists "$ENV_NAME" \
    || { echo "✗ 없는 환경: $ENV_NAME"; echo "  사용 가능:"; list_envs; exit 1; }

[ "$(docker container inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ] \
    || { echo "✗ 컨테이너 꺼짐 → scripts/container_up.sh 먼저"; exit 1; }
docker exec $C pgrep -f "[e]nv/script/" >/dev/null 2>&1 && { echo "⚠ 이미 실행 중"; exit 0; }

mkdir -p "$REPO_DIR/logs"

# Isaac Sim 이 번들한 ROS 2 를 쓴다. 시스템 ROS 설치는 필요 없고, 이 세 변수만
# 잡아 주면 시뮬레이션 프로세스 안에서 rclpy 가 import 된다.
# jazzy 를 쓰는 이유는 컨테이너가 Ubuntu 24.04 라 짝이 맞기 때문이다 (humble 도 된다).
ROS_BUNDLE=/isaac-sim/exts/isaacsim.ros2.bridge/${ROS_DISTRO_BUNDLE:-jazzy}

docker exec -d $C bash -c \
    "export PYTHONPATH=$ROS_BUNDLE/rclpy:\$PYTHONPATH \
        LD_LIBRARY_PATH=$ROS_BUNDLE/lib:\$LD_LIBRARY_PATH \
        RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
     /workspace/isaaclab/_isaac_sim/python.sh -u /workspace/franka_robolab_sim/env/script/run.py ${ENV_NAME} \
        --headless $* > /workspace/franka_robolab_sim/logs/sim.log 2>&1"

echo "기동 중: ${ENV_NAME}  (Isaac Sim 첫 실행은 셰이더 컴파일로 2~5분 걸립니다)"
for _ in $(seq 1 90); do
    sleep 5
    if grep -q "\[env\] 준비 완료" "$REPO_DIR/logs/sim.log" 2>/dev/null; then
        echo "✓ 준비 완료 → http://$(hostname -I | awk '{print $1}'):8003"
        exit 0
    fi
    if grep -qE "Traceback|\[env\] 종료:|✗ 환경" "$REPO_DIR/logs/sim.log" 2>/dev/null; then
        echo "✗ 기동 실패 — 로그 마지막 20줄:"; tail -20 "$REPO_DIR/logs/sim.log"; exit 1
    fi
done
echo "⚠ 시간 초과 — 로그 확인: scripts/logs.sh"
tail -20 "$REPO_DIR/logs/sim.log" 2>/dev/null || true
