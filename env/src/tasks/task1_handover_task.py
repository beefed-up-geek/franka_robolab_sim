# SPDX-License-Identifier: Apache-2.0
"""task1 — 공구 건네주기.

흰색 워크스페이스 위 공구 3점(망치 7·무선드릴·가위)을 집어, 작업대 -Y 측에 서서
상판 위로 손을 뻗은 작업자의 손바닥 받침에 올려놓는다. 상판의 작업자 구역
(y <= -0.40)은 노랑/검정 주의 테이프로 구분되어 있다 — 도면 v3
(_tool_viewer/task1_plan.html) 이 이 배치의 근거다.

작업자·손바닥 받침은 씬이 아니라 Task1HandoverWorldCfg 가 스폰한다
(컨베이어·통을 world_assets 로 뺀 task3 와 같은 이유 — payload 안 정적
콜라이더는 PhysX 에 등록되지 않는다).

성공 종료 조건을 넣지 않은 것은 task3 와 같은 이유다(README 참고) — RobolabEnv
는 종료 시 env 를 freeze 시키므로 시간 초과만 남긴다.
"""
from dataclasses import dataclass
from pathlib import Path

import isaaclab.envs.mdp as mdp
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from robolab.core.scenes.utils import import_scene
from robolab.core.task.task import Task

SCENE_PATH = str(
    Path(__file__).resolve().parents[2] / "asset" / "scenes" / "task1_handover.usda"
)

# import_scene 이 이 목록의 프림만 씬 엔티티로 만든다 (task3 와 동일 규칙).
# 가위는 뺐다 — 두께 15mm 라 패드가 거의 다 닫힌 채 마찰로만 물려서, 운반 중
# 미끄러져 빠진다(실측: 접촉력 0 인 채 반쯤 가다 낙하). 집을 수 있는 공구만 남긴다.
TOOLS = [
    "hammer_7",          # HANDAL 망치 — 손잡이 파지
    "cordless_drill",    # YCB 무선드릴 — 가장 부피가 큼
]

CONTACT_OBJECTS = [*TOOLS, "table"]


@configclass
class Task1HandoverTaskTerminations:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@dataclass
class Task1HandoverTask(Task):
    contact_object_list = CONTACT_OBJECTS
    scene = import_scene(SCENE_PATH, CONTACT_OBJECTS)
    terminations = Task1HandoverTaskTerminations
    instruction = {
        "default": "Hand the tool to the worker",
        "hammer": "Hand the hammer to the worker",
        "drill": "Hand the cordless drill to the worker",
        "scissors": "Hand the scissors to the worker",
    }
    episode_length_s: int = 86400
    attributes = ["semantics"]
