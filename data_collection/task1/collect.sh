#!/usr/bin/env bash
# task1 공구 전달 시연 수집 — 컨테이너 안에서 Isaac 번들 ROS 로 돈다.
#
#   ./data_collection/task1/collect.sh --per-tool 50    # _data/task1/<날짜시간> 에 저장
#
# 시뮬레이션이 먼저 떠 있어야 한다: ./scripts/task1.sh
set -euo pipefail

C=franka_robolab_sim
B=/isaac-sim/exts/isaacsim.ros2.bridge/${ROS_DISTRO_BUNDLE:-jazzy}

TTY=$([ -t 1 ] && echo -t || true)

# 고아 수집기 정리 — timeout/Ctrl-C 는 docker exec 클라이언트만 죽인다 (task3 교훈).
docker exec $C pkill -f "data_collection/task1/collect.py" 2>/dev/null && sleep 1 || true
trap "docker exec $C pkill -f data_collection/task1/collect.py 2>/dev/null || true" EXIT

docker exec $TTY $C bash -c \
    "export PYTHONPATH=$B/rclpy:\$PYTHONPATH \
        LD_LIBRARY_PATH=$B/lib:\$LD_LIBRARY_PATH \
        RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
     /isaac-sim/python.sh /workspace/franka_robolab_sim/data_collection/task1/collect.py $*"
