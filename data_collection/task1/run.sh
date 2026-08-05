#!/usr/bin/env bash
# task1 공구 전달 — 컨테이너 안 Isaac 번들 ROS 로 돈다 (task3 run.sh 와 동일 구조).
#
#   ./data_collection/task1/run.sh --tool hammer --yaw 90 --speed 0.2
#
# 시뮬레이션이 먼저 떠 있어야 한다: ./scripts/task1.sh
set -euo pipefail

C=franka_robolab_sim
B=/isaac-sim/exts/isaacsim.ros2.bridge/${ROS_DISTRO_BUNDLE:-jazzy}
TTY=$([ -t 1 ] && echo -t || true)

docker exec $C pkill -f "data_collection/task1/deliver.py" 2>/dev/null && sleep 1 || true
trap "docker exec $C pkill -f data_collection/task1/deliver.py 2>/dev/null || true" EXIT

docker exec $TTY $C bash -c \
    "export PYTHONPATH=$B/rclpy:\$PYTHONPATH \
        LD_LIBRARY_PATH=$B/lib:\$LD_LIBRARY_PATH \
        RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
     /isaac-sim/python.sh /workspace/franka_robolab_sim/data_collection/task1/deliver.py $*"
