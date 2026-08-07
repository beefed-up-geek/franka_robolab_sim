# SPDX-License-Identifier: Apache-2.0
"""task2 test — 발전기 DC 전선을 배터리 단자에 연결 (작업자 팔 침입).

배치·기물은 train 과 완전히 동일하다. 유일한 차이는 **작업자 팔**(worker_arm):
로봇이 커넥터를 들어 옮기는 동안 책상 -y 모서리(작업자 자리)에서 팔이 작업
영역에 들어왔다 나간다 (runner 의 ArmIntruder 상태기계가 구동, 시드 고정
난수로 시점·레인·체류를 샘플링). 변수를 팔 하나로 고정했으므로 train 대비
성공률·충돌·시간 변화가 곧 침입 조건의 효과다.
"""
from dataclasses import dataclass
from pathlib import Path

import isaaclab.envs.mdp as mdp
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from robolab.core.scenes.utils import import_scene
from robolab.core.task.task import Task

SCENE_PATH = str(
    Path(__file__).resolve().parents[2] / "asset" / "scenes" / "task2_test_charging.usda"
)

# worker_arm: 킨매틱 침입 팔 (runner 의 ArmIntruder 가 구동) — TOOLS 등록으로
# 강체 뷰(자세 기록)와 ROS objects 발행을, CONTACT_OBJECTS 로 그리퍼 접촉
# 계측(status.contact["worker_arm"])을 얻는다. train 에는 없는 프림이라
# runner 의 팔 블록은 test 에서만 활성이다.
TOOLS = ["battery", "connector_red", "connector_black", "worker_arm"]

CONTACT_OBJECTS = [*TOOLS, "table"]


@configclass
class Task2TestTaskTerminations:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@dataclass
class Task2TestTask(Task):
    contact_object_list = CONTACT_OBJECTS
    scene = import_scene(SCENE_PATH, CONTACT_OBJECTS)
    terminations = Task2TestTaskTerminations
    instruction = {
        "default": "Plug the charging cable into the battery",
    }
    episode_length_s: int = 86400
    attributes = ["semantics"]
