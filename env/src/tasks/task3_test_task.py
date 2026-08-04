# SPDX-License-Identifier: Apache-2.0
"""task3 평가 태스크 — 팽창·파열된 불량품이 섞여 흐른다.

train 과 설비는 완전히 같고(씬이 _can_workcell.usda 를 공유한다) 흐르는 물건만
다르다. 정상품 5종에 그 짝인 파열품 5종을 더해 10종이 돌아간다.

짝을 맞춘 이유는 정책이 **결함 자체를** 보게 하기 위해서다. 불량품만 라벨이 다르면
그림만 외워도 골라낼 수 있다. 파열품은 부푼 뚜껑·찌그러진 옆면·뜯긴 구멍만 다르고
텍스처는 같다. 게다가 아래 뚜껑이 볼록해 벨트 위에서 비스듬히 기울기 때문에,
train 에서 보지 못한 파지 자세가 된다.

성공 종료 조건을 넣지 않은 것은 의도적이다. RobolabEnv 는 에피소드가 종료되면
env 를 freeze 시켜 액션을 0 으로 만들어버리므로(README 참고), 하나 담았다고
종료되면 그 순간부터 조작이 멈춘다. 사람이 계속 조작하는 샌드박스가 목적이라
시간 초과만 남겼다.
"""
from dataclasses import dataclass
from pathlib import Path

import isaaclab.envs.mdp as mdp
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from robolab.core.scenes.utils import import_scene
from robolab.core.task.task import Task

SCENE_PATH = str(
    Path(__file__).resolve().parents[2] / "asset" / "scenes" / "task3_test.usda"
)

# 벨트에 실릴 물체는 모두 여기에 있어야 한다. RoboLab 의 import_scene 이 이 목록에
# 있는 프림만 씬 엔티티로 만들기 때문에, 빠지면 화면에 그려지기만 하고 컨베이어가
# 손댈 수 없다. 씬 USD 의 프림 이름과 정확히 같아야 한다.
CANS = [
    "corn_can",          # Ø71x58mm   표준 캔 (노랑, 옥수수 사진)
    "normal_can",        # Ø71x58mm   단색 적색 라벨 (tools/make_cans.py)
    "sardine_can",       # Ø71x58mm   마린 블루 라벨 — normal_can 과 물성 동일
    "fig_can",           # Ø71x58mm   가지색 라벨 — normal_can 과 물성 동일
    # 위 넷의 파열품. 정상품과 **같은 라벨**을 쓰고 형상만 다르다 — 라벨 그림이
    # 아니라 부푼 뚜껑·찌그러진 옆면·뜯긴 구멍을 보고 골라내야 한다.
    "corn_can_burst",          # Ø71x82.8mm
    "burst_can",               # Ø71x83.0mm
    "sardine_can_burst",       # Ø71x83.0mm
    "fig_can_burst",           # Ø71x83.0mm
]

# "table" 은 반드시 있어야 한다 — RoboLab 의 접촉 프레디킷이 gripper__table
# 센서를 이름으로 찾는다. grey_bin 은 씬 USD 가 아니라 world_assets.py 에서 별도
# 엔티티로 스폰하므로 여기 넣지 않는다 (넣으면 import_scene 이 프림을 찾지 못한다).
CONTACT_OBJECTS = [*CANS, "table"]


@configclass
class Task3TestPickPlaceCanTaskTerminations:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@dataclass
class Task3TestPickPlaceCanTask(Task):
    contact_object_list = CONTACT_OBJECTS
    scene = import_scene(SCENE_PATH, CONTACT_OBJECTS)
    terminations = Task3TestPickPlaceCanTaskTerminations
    instruction = {
        "default": "Pick up the cans from the conveyor and put them in the bin",
        "vague": "Move the cans into the bin",
        "specific": "Grasp each can as it travels along the conveyor belt and place it inside the grey bin on the table, including the swollen and burst ones",
    }
    # 사람이 조작하는 샌드박스라 넉넉히 잡는다 (24시간 — 1시간이었을 때 장시간 수집 중 매시간 soft 리셋이 시도를 끊었다).
    episode_length_s: int = 86400
    attributes = ["semantics"]
