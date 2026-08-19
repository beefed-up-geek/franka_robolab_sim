#!/usr/bin/env bash
# 컨테이너 올리기 (없으면 생성, 있으면 시작)
set -euo pipefail

C=franka_robolab_sim
IMAGE=franka_robolab_sim:latest
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# RoboLab 프레임워크 — 2026-08-19 부터 이 레포 안(robolab/)에 산다. 컨테이너 안
# 경로(/workspace/robolab)는 그대로다 — 이미지의 editable pip 설치가 그 경로를
# 가리키므로 마운트 목적지를 바꾸면 import 가 깨진다.
ROBOLAB_DIR="${ROBOLAB_DIR:-$REPO_DIR/robolab}"

if [ "$(docker container inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ]; then
    echo "✓ 이미 실행 중: $C"; exit 0
fi
if docker container inspect $C >/dev/null 2>&1; then
    docker start $C >/dev/null && echo "✓ 컨테이너 시작: $C"; exit 0
fi

[ -d "$ROBOLAB_DIR" ] || { echo "✗ RoboLab 저장소가 없습니다: $ROBOLAB_DIR"; exit 1; }
mkdir -p "$HOME/.cache/ov" "$HOME/.cache/kit" "$REPO_DIR/logs"

# RoboLab 은 로봇 설정과 파이썬 패키지 때문에 필요하다.
# 씬 자산은 이 저장소의 env/asset 안에 자립적으로 들어 있어 마운트 배치에
# 의존하지 않는다.
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
