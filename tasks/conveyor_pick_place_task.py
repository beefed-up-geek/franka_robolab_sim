# SPDX-License-Identifier: Apache-2.0
"""컨베이어 pick-and-place 태스크.

벨트를 타고 흘러오는 블록을 사람이 텔레오퍼레이션으로 집어 그릇에 담는다.

성공 종료 조건을 넣지 않은 것은 의도적이다. RobolabEnv 는 에피소드가 종료되면
env 를 freeze 시켜 액션을 0 으로 만들어버리므로(README 참고), 블록 하나를 담았다고
종료되면 그 순간부터 조작이 멈춘다. 계속 조작하는 샌드박스가 목적이라 시간 초과만
남겼다.
"""
from dataclasses import dataclass
from pathlib import Path

import isaaclab.envs.mdp as mdp
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from robolab.core.scenes.utils import import_scene
from robolab.core.task.task import Task

SCENE_PATH = str(
    Path(__file__).resolve().parents[1] / "assets" / "scenes" / "conveyor_pick_place.usda"
)

BLOCKS = [f"block_{i}" for i in range(6)]

# 벨트에 실릴 수 있는 물체는 모두 여기에 있어야 한다.
# import_scene 이 이 목록에 있는 프림만 씬 엔티티로 만들기 때문에, 여기 빠진
# 물체는 씬에 그려지기만 하고 컨베이어가 손댈 수 없다.
# (can_a 는 "블록이 아닌 물체도 실리는가" 를 확인하려고 둔 원기둥이다)
CARGO = [*BLOCKS, "can_a"]

# "table" 은 반드시 있어야 한다 — RoboLab 의 접촉 프레디킷이 gripper__table
# 센서를 이름으로 찾는다.
CONTACT_OBJECTS = [*CARGO, "bowl", "table"]


@configclass
class ConveyorPickPlaceTerminations:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@dataclass
class ConveyorPickPlaceTask(Task):
    contact_object_list = CONTACT_OBJECTS
    scene = import_scene(SCENE_PATH, CONTACT_OBJECTS)
    terminations = ConveyorPickPlaceTerminations
    instruction = {
        "default": "Pick up the blocks from the conveyor and place them in the bowl",
        "vague": "Move the blocks into the bowl",
        "specific": "Grasp each colored block as it travels along the conveyor belt "
                    "and place it inside the bowl on the table",
    }
    # 사람이 조작하는 샌드박스라 넉넉히 잡는다 (1시간).
    episode_length_s: int = 3600
    attributes = ["semantics"]
