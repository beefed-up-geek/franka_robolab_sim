# SPDX-License-Identifier: Apache-2.0
"""흰색 테이블 pick-and-place 태스크.

RoboLab 벤치마크 태스크(robolab/tasks/benchmark/banana_in_bowl_task.py)와 같은
구조지만, 씬만 이 저장소의 흰색 테이블로 바꿨다. 성공 판정·서브태스크는
RoboLab 의 조합형 프레디킷을 그대로 쓴다 — 텔레오퍼레이션으로 물체를 그릇에
넣으면 success 가 뜬다.
"""
from dataclasses import dataclass
from pathlib import Path

import isaaclab.envs.mdp as mdp
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from robolab.core.scenes.utils import import_scene
from robolab.core.task.conditionals import object_in_container, pick_and_place
from robolab.core.task.task import Task

# 이 저장소 안의 씬을 절대경로로 넘긴다. import_scene 은 상대경로일 때만
# RoboLab 의 SCENE_DIR 을 뒤지므로, 외부 씬은 절대경로로 줘야 한다.
SCENE_PATH = str(Path(__file__).resolve().parents[1] / "assets" / "scenes" / "white_table_pick_place.usda")

# "table" 이 반드시 들어가야 한다 — pick_and_place 서브태스크의 gripper_hit_table
# 프레디킷이 gripper__table 접촉 센서를 요구한다. 빠지면 첫 스텝에서 ValueError.
CONTACT_OBJECTS = ["banana", "bowl", "table"]


@configclass
class WhiteTablePickPlaceTerminations:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    success = DoneTerm(
        func=object_in_container,
        params={
            "object": "banana",
            "container": "bowl",
            "gripper_name": "gripper",
            "tolerance": 0.0,
            "require_contact_with": True,
            "require_gripper_detached": True,
        },
    )


@dataclass
class WhiteTablePickPlaceTask(Task):
    contact_object_list = CONTACT_OBJECTS
    scene = import_scene(SCENE_PATH, CONTACT_OBJECTS)
    terminations = WhiteTablePickPlaceTerminations
    instruction = {
        "default": "Pick up the banana and place it in the bowl",
        "vague": "Put the fruit in the bowl",
        "specific": "Grasp the yellow banana on the white table and place it inside the bowl",
    }
    # 텔레오퍼레이션은 사람이 조작하므로 벤치마크(50s)보다 넉넉하게 준다.
    episode_length_s: int = 600
    attributes = ["semantics"]

    subtasks = [
        pick_and_place(
            object=["banana"],
            container="bowl",
            logical="all",
            score=1.0,
        )
    ]
