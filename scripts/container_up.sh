#!/usr/bin/env bash
# 컨테이너 올리기 (없으면 생성, 있으면 시작)
set -euo pipefail

C=franka_robolab_sim
IMAGE=franka_robolab_sim:latest
ROBOLAB_DIR="${ROBOLAB_DIR:-$HOME/robolab}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "$(docker container inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ]; then
    echo "✓ 이미 실행 중: $C"; exit 0
fi
if docker container inspect $C >/dev/null 2>&1; then
    docker start $C >/dev/null && echo "✓ 컨테이너 시작: $C"; exit 0
fi

[ -d "$ROBOLAB_DIR" ] || { echo "✗ RoboLab 저장소가 없습니다: $ROBOLAB_DIR"; exit 1; }
mkdir -p "$HOME/.cache/ov" "$HOME/.cache/kit" "$REPO_DIR/logs"

# 두 저장소는 반드시 /workspace 아래 형제로 마운트한다 —
# assets/scenes/*.usda 의 payload 상대경로(../../../robolab/...)가 이 배치에 의존한다.
docker run -d -it --name $C \
    --runtime nvidia --gpus all \
    --net host \
    -e ACCEPT_EULA=Y -e OMNI_KIT_ACCEPT_EULA=YES \
    -v "$ROBOLAB_DIR":/workspace/robolab \
    -v "$REPO_DIR":/workspace/franka_robolab_sim \
    -v "$HOME/.cache/ov":/root/.cache/ov \
    -v "$HOME/.cache/kit":/isaac-sim/kit/cache \
    --restart unless-stopped \
    "$IMAGE" >/dev/null && echo "✓ 컨테이너 생성·시작: $C"
