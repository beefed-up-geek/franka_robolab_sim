# SPDX-License-Identifier: Apache-2.0
"""task2 train — 발전기 DC 전선을 배터리 단자에 연결.

흰색 워크스페이스 위에 SAM3D 로 복원한 자동차 배터리(H5 실측 스케일)와 휴대
발전기를 올려 둔 상태다. 전선·플러그(파지 대상)와 + 단자 안착 판정은 다음
단계에서 붙는다 — 그때 TOOLS 에 플러그가 들어온다.

배치 근거는 씬 usda 의 doc 참고 (기본 뷰 기준 배터리 오른쪽 · 발전기 왼쪽,
중심 간 0.42m).
"""
from dataclasses import dataclass
from pathlib import Path

import isaaclab.envs.mdp as mdp
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from robolab.core.scenes.utils import import_scene
from robolab.core.task.task import Task

SCENE_PATH = str(
    Path(__file__).resolve().parents[2] / "asset" / "scenes" / "task2_train_charging.usda"
)

# 구축 1단계 — 아직 파지 대상이 없다. 플러그가 붙으면 여기 들어온다.
# 배터리는 강체 씬 엔티티로 등록한다 (접촉 센서 활성화에 강체가 최소 하나
# 필요하고, ROS objects 로 자세가 나가야 다음 단계의 단자 목표 계산이 된다).
TOOLS = ["battery", "connector_red", "connector_black"]

CONTACT_OBJECTS = [*TOOLS, "table"]


@configclass
class Task2TrainTaskTerminations:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@dataclass
class Task2TrainTask(Task):
    contact_object_list = CONTACT_OBJECTS
    scene = import_scene(SCENE_PATH, CONTACT_OBJECTS)
    terminations = Task2TrainTaskTerminations
    instruction = {
        "default": "Plug the charging cable into the battery",
    }
    episode_length_s: int = 86400
    attributes = ["semantics"]
