#!/usr/bin/env bash
# 이미지 빌드: robolab 베이스(없으면 생성) → franka_robolab_sim
set -euo pipefail

ROBOLAB_DIR="${ROBOLAB_DIR:-$HOME/robolab}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_IMAGE="robolab:teleop"
IMAGE="franka_robolab_sim:latest"

if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
    echo "▶ 베이스 이미지 빌드: $BASE_IMAGE (IsaacLab 2.3 / IsaacSim 5.1) — ~42GB, 수 분 소요"
    [ -d "$ROBOLAB_DIR" ] || { echo "✗ RoboLab 저장소가 없습니다: $ROBOLAB_DIR"; exit 1; }
    # 태그는 위치인자로, 스택은 --isaac51 플래그로 준다.
    # (build_docker.sh 는 ISAACLAB_TAG 를 하드코딩하므로 환경변수로는 못 바꾼다)
    (cd "$ROBOLAB_DIR" && ./docker/build_docker.sh teleop --isaac51)
else
    echo "✓ 베이스 이미지 존재: $BASE_IMAGE"
fi

echo "▶ 빌드: $IMAGE"
docker build -t "$IMAGE" \
    --network=host \
    --build-arg "ROBOLAB_IMAGE=$BASE_IMAGE" \
    -f "$REPO_DIR/docker/Dockerfile" \
    "$REPO_DIR"

echo "✓ 완료: $IMAGE"
