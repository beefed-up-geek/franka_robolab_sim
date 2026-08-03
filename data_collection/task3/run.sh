#!/usr/bin/env bash
# task3 시연 수집 — 컨테이너 안에서 Isaac 번들 ROS 로 돈다.
#
#   ./data_collection/task3/run.sh --episodes 20            # _data/task3/<날짜시간> 에 저장
#   ./data_collection/task3/run.sh --episodes 20 --out ...  # 다른 곳에 저장
#
# 시뮬레이션이 먼저 떠 있어야 한다: ./scripts/task3_train.sh
set -euo pipefail

C=franka_robolab_sim
B=/isaac-sim/exts/isaacsim.ros2.bridge/${ROS_DISTRO_BUNDLE:-jazzy}

# TTY 가 있을 때만 -t 를 준다. 없는데 주면 "cannot attach stdin to a TTY-enabled
# container" 로 죽는다 (ssh 로 원격 실행할 때 걸린다).
TTY=$([ -t 1 ] && echo -t || true)

# 이전 수집기를 반드시 정리한다. timeout 이나 Ctrl-C 는 docker exec **클라이언트**만
# 죽이고 컨테이너 안 프로세스는 남긴다 — 고아 수집기 7개가 동시에 명령을 쏘며
# 로봇을 줄다리기시킨 적이 있다. 종료 시에도 같은 이유로 trap 으로 죽인다.
docker exec $C pkill -f "data_collection/task3/collect.py" 2>/dev/null && sleep 1 || true
trap "docker exec $C pkill -f data_collection/task3/collect.py 2>/dev/null || true" EXIT

docker exec $TTY $C bash -c \
    "export PYTHONPATH=$B/rclpy:\$PYTHONPATH \
        LD_LIBRARY_PATH=$B/lib:\$LD_LIBRARY_PATH \
        RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
     /isaac-sim/python.sh /workspace/franka_robolab_sim/data_collection/task3/collect.py $*"
