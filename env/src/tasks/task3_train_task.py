# SPDX-License-Identifier: Apache-2.0
"""task3 학습 태스크 — 정상품 3개 정적 배치 (배치 모드).

벨트는 **정지** 상태이고, 정상 캔 3개가 벨트 위 무작위 위치에 놓인다
(envs.py 의 batch=3 — conveyor.py 배치 모드). 셋을 모두 통에 담으면 환경이
새 3개를 무작위 위치·무작위 종류로 재배치한다. 결함이 있는 물건은 하나도
나오지 않으므로, 여기서 모은 시연으로 학습한 정책은 파열품을 본 적이 없다.

움직이는 벨트(v3~v7)에서는 가변 속도 때문에 올바른 요격점이 한 프레임 관측으로
정해지지 않아(같은 장면에 여러 정답) 폐루프 성공률이 0 으로 무너졌다 — 정적
배치는 그 다봉성을 제거한 재설계다.

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
    Path(__file__).resolve().parents[2] / "asset" / "scenes" / "task3_train.usda"
)

# 벨트에 실릴 물체는 모두 여기에 있어야 한다. RoboLab 의 import_scene 이 이 목록에
# 있는 프림만 씬 엔티티로 만들기 때문에, 빠지면 화면에 그려지기만 하고 컨베이어가
# 손댈 수 없다. 씬 USD 의 프림 이름과 정확히 같아야 한다.
CANS = [
    "corn_can",          # Ø71x58mm   표준 캔 (노랑, 옥수수 사진)
    "normal_can",        # Ø71x58mm   단색 적색 라벨 (tools/make_cans.py)
    # 아래 둘은 normal_can 과 **치수·질량·충돌 형상이 완전히 같고 라벨만 다르다.**
    # 잡는 법은 그대로 두고 보이는 것만 늘리려는 것이다 — 같은 동작으로 집히는
    # 서로 다른 라벨을 보여 주면 정책이 캔 종류에 동작을 결부시키지 않는다
    # (tools/can_designs.py).
    "sardine_can",       # Ø71x58mm   마린 블루 · 정어리
    "fig_can",           # Ø71x58mm   가지색 · 무화과
]

# "table" 은 반드시 있어야 한다 — RoboLab 의 접촉 프레디킷이 gripper__table
# 센서를 이름으로 찾는다. grey_bin 은 씬 USD 가 아니라 world_assets.py 에서 별도
# 엔티티로 스폰하므로 여기 넣지 않는다 (넣으면 import_scene 이 프림을 찾지 못한다).
CONTACT_OBJECTS = [*CANS, "table"]


@configclass
class Task3TrainPickPlaceCanTaskTerminations:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@dataclass
class Task3TrainPickPlaceCanTask(Task):
    contact_object_list = CONTACT_OBJECTS
    scene = import_scene(SCENE_PATH, CONTACT_OBJECTS)
    terminations = Task3TrainPickPlaceCanTaskTerminations
    instruction = {
        "default": "Pick up the cans from the conveyor and put them in the bin",
        "vague": "Move the cans into the bin",
        "specific": "Grasp each of the three cans resting on the stopped conveyor belt and place it inside the grey bin on the table",
    }
    # 사람이 조작하는 샌드박스라 넉넉히 잡는다 (24시간 — 1시간이었을 때 장시간 수집 중 매시간 soft 리셋이 시도를 끊었다).
    episode_length_s: int = 86400
    attributes = ["semantics"]
