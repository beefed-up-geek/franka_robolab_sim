# SPDX-License-Identifier: Apache-2.0
"""환경 레지스트리 — 이름 하나로 어떤 환경이든 띄운다 (env/script/run.py).

Isaac Sim 을 띄우기 **전에** import 되므로 표준 라이브러리 밖의 것을
가져오면 안 된다. 월드 cfg 가 클래스가 아니라 이름 문자열인 것도 그래서다
— run.py 가 앱 기동 후 franka_env.world_assets 에서 getattr 로 푼다.

환경마다 다른 것은 이 다섯 값뿐이다: 태스크 클래스·월드·컨베이어 모드·
기본 인자·설명. 태스크의 내용(씬·물체·판정·지시문)은 env/src/tasks/ 의
태스크 정의가 갖는다. **새 환경 = 태스크 정의 1개 + 여기 한 줄.**

기본 인자(defaults)는 franka_env.cli 의 인자 기본값을 환경별로 덮어쓴다 —
예전에 scripts/task3_train.sh 같은 셸 래퍼에 박혀 있던 실험 조건이 여기로
왔다. 실행 시 CLI 인자가 다시 그 위를 덮는다.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class EnvSpec:
    task: str           # env/src/tasks 의 태스크 클래스 이름
    world: str          # franka_env.world_assets 의 cfg 클래스 이름
    conveyor: str       # franka_env.cli.CONVEYOR_MODES 중 하나
    description: str
    defaults: dict = field(default_factory=dict)


ENVS: dict[str, EnvSpec] = {
    "task1": EnvSpec(
        task="Task1HandoverTask",
        world="Task1HandoverWorldCfg",
        conveyor="none",
        description="공구 3종 · 작업자 핸드오버 · 주의 테이프 구역",
        defaults={"grip_force": 25.0, "can_mass": 0.05},
    ),
    "task2_train": EnvSpec(
        task="Task2TrainTask",
        world="Task2ChargingWorldCfg",
        conveyor="none",
        description="발전기 플러그를 배터리 단자에 연결 (수집)",
        defaults={"grip_force": 25.0},
    ),
    "task2_test": EnvSpec(
        task="Task2TestTask",
        world="Task2ChargingWorldCfg",
        conveyor="none",
        description="train 동일 배치 + 운반 중 작업자 팔 침입 (평가)",
        defaults={"grip_force": 25.0},
    ),
    "task3_train": EnvSpec(
        task="Task3TrainPickPlaceCanTask",
        world="CanSortingWorldCfg",
        conveyor="script",
        description="정상 캔 3개 정적 배치 · 벨트 정지 · 다 담으면 재배치 (수집)",
        defaults={"grip_force": 25.0, "belt_speed": 0.0, "batch": 3,
                  "defect_ratio": 0.0, "belt_jitter": 0.0},
    ),
    "task3_test": EnvSpec(
        task="Task3TestPickPlaceCanTask",
        world="CanSortingWorldCfg",
        conveyor="script",
        description="캔 3개(정상+파열 1~2 혼합) 정적 배치 · 정상만 담으면 초기화 (평가)",
        defaults={"grip_force": 25.0, "belt_speed": 0.0, "batch": 3,
                  "defect_ratio": 0.2, "belt_jitter": 0.0},
    ),
}
