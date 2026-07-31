# RoboLab 이미지 위에 웹 서버 의존성만 얹는다.
# 베이스(robolab:*)는 ~/robolab/docker/build_docker.sh 로 먼저 만들어야 한다.
ARG ROBOLAB_IMAGE=robolab:teleop
FROM ${ROBOLAB_IMAGE}

# Isaac Sim 번들 파이썬에 설치해야 한다 — 시스템 python3 가 아니다.
RUN /workspace/isaaclab/_isaac_sim/python.sh -m pip install --no-cache-dir aiohttp

# 이 저장소는 런타임에 볼륨으로 붙는다 (scripts/container_up.sh 참고).
# 베이스 이미지에 PYTHONPATH 가 없으므로 이어붙이지 않고 그냥 설정한다.
ENV PYTHONPATH=/workspace/franka_robolab_sim
ENV OMNI_KIT_ACCEPT_EULA=YES
ENV ACCEPT_EULA=Y
WORKDIR /workspace/franka_robolab_sim

CMD ["sleep", "infinity"]
