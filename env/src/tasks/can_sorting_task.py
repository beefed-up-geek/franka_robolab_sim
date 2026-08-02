# SPDX-License-Identifier: Apache-2.0
"""컨베이어 통조림 분류 태스크.

벨트를 타고 흘러오는 통조림을 집어 회색 통(grey_bin)에 담는다.

블록 대신 통조림을 쓴 이유는 높이가 제각각(33 / 58 / 83mm)이기 때문이다. 같은 크기
블록만 흘려보내면 파지 높이가 늘 같아서 시연 데이터가 단조로워지는데, 통조림은
매번 그리퍼 높이를 맞춰야 해서 VLA 학습에 쓸 만한 변화가 생긴다. 반면 바닥
지름(70mm 안팎)은 거의 같아 벨트 위에서는 안정적으로 서 있는다.

성공 종료 조건을 넣지 않은 것은 의도적이다. RobolabEnv 는 에피소드가 종료되면
env 를 freeze 시켜 액션을 0 으로 만들어버리므로(README 참고), 하나 담았다고
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
    Path(__file__).resolve().parents[2] / "asset" / "scenes" / "can_sorting.usda"
)

# 벨트에 실릴 물체는 모두 여기에 있어야 한다. RoboLab 의 import_scene 이 이 목록에
# 있는 프림만 씬 엔티티로 만들기 때문에, 빠지면 화면에 그려지기만 하고 컨베이어가
# 손댈 수 없다. 씬 USD 의 프림 이름과 정확히 같아야 한다.
CANS = [
    "canned_tuna",
    "corn_can",
    "green_beans_can",
    "canned_peaches",
    "pineapple_slices_can",
    "tomato_sauce_can",
    "canned_mushrooms",
]

# "table" 은 반드시 있어야 한다 — RoboLab 의 접촉 프레디킷이 gripper__table
# 센서를 이름으로 찾는다.
# grey_bin 은 씬 USD 가 아니라 world_assets.py 에서 별도 엔티티로 스폰하므로
# 여기 넣지 않는다 (넣으면 import_scene 이 프림을 찾지 못한다).
CONTACT_OBJECTS = [*CANS, "table"]


@configclass
class CanSortingTerminations:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@dataclass
class CanSortingTask(Task):
    contact_object_list = CONTACT_OBJECTS
    scene = import_scene(SCENE_PATH, CONTACT_OBJECTS)
    terminations = CanSortingTerminations
    instruction = {
        "default": "Pick up the cans from the conveyor and put them in the bin",
        "vague": "Move the cans into the bin",
        "specific": "Grasp each can as it travels along the conveyor belt "
                    "and place it inside the grey bin on the table",
    }
    # 사람이 조작하는 샌드박스라 넉넉히 잡는다 (1시간).
    episode_length_s: int = 3600
    attributes = ["semantics"]
