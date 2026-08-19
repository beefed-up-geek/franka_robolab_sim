# SPDX-License-Identifier: Apache-2.0
"""task2 test — 작업자 팔 배치 (runner 구동).

작업자는 절단된 책상 앞모서리(x=0.65) 밖에 서 있고, **팔꿈치부터 손까지만**
보인다. 환경이 초기화되면 팔은 수평으로 책상 밖에서 대기하다가 **1초 뒤 쑥
들어와 그 자리에 머무른다.** 자리는 초기화마다 **무작위**로 다시 뽑는다.

들어왔다 나가기를 반복하던 상태기계는 걷어냈다. 에피소드가 "붉은 플러그를
꽂으면 성공, 팔에 닿으면 실패" 로 짧아졌으므로 팔이 도중에 사라지면 조건이
시점에 따라 흔들린다. **한 번 들어와 머무르는** 것이 매 에피소드를 같은
성격의 문제로 만든다.

진입은 초기화 4초 뒤다. 그맘때 로봇은 이미 플러그를 집어 옮기는 중이라,
"작업하는 옆으로 사람 팔이 쑥 들어오는" 상황이 된다 — 처음부터 놓여 있으면
정책이 출발선에서부터 그 자리를 피해 가므로 침입이라기보다 정적 장애물이다.

팔 강체의 기하는 팔꿈치 원점에서 -X 로 뻗으므로(worker_arm.usda) 수평 자세가
곧 항등 방향이다 — 회전을 쓰지 않으니 배치가 위치 하나로 끝난다. 매 스텝
write_root_pose 재기록 — 자성 홀드와 같은 검증된 패턴이라 리셋 폭발이 없다.
(킨매틱 플래그는 프레임워크 import_scene 이 RigidObject 로 등록하지 않아
못 쓴다 — 동적+재기록으로 해결.)

높이는 **0.30 하나**로 고정한다 (배치: 배터리 (0.36,-0.48) z180, 단자 포스트
x 0.265·0.458 / y -0.43 / 상단 z 0.223, 운반 높이 z≈0.31). 팔대가 운반
경로(플러그→단자, x 0.27~0.46 대역)를 운반 높이에서 정면으로 가로지르므로,
그대로 직진하면 부딪힌다. 낮은 레인은 들어 올린 뒤 위로 지나가 버려 평가
조건이 되지 못했다 — 변수를 **좌우 위치 하나**로 줄였다.

## 시드 1~5 — 배터리와 발전기 **사이**를 좌우로 훑는 다섯 자리

`--arm-seed N` (N=1~5) 을 주면 손바닥 위치가 아래 표로 **고정**된다.
배터리(y=-0.48)와 발전기(y=+0.5) 사이가 곧 운반 통로이고, 그 통로를 y
-0.30 에서 +0.10 까지 균등하게 다섯으로 나눈 것이다. x·z 는 전부 같다.

시드를 주지 않으면(0) **리셋마다 무작위**로 뽑는다 — 같은 높이에서 y 만
통로 전역으로 흩어진다. 평가 조건을 재현하려면 1~5 를 쓴다.
"""
from __future__ import annotations

import time

import numpy as np
import torch

_L = 0.385              # 팔꿈치 원점 -> 손바닥 중심 (손끝은 0.52)
# 대기 위치의 x [m, 팔꿈치 원점 기준]. 손바닥이 책상 앞모서리(0.65) 바로 바깥
# 에 오는 값이라, 화면에는 팔 끝만 걸쳐 보이고 작업 영역은 비어 있다.
_PARK_X = 1.05
_ENTER_DELAY = 4.0      # 초기화 후 진입까지 [s] (사용자 지정)
_ENTER_T = 0.45         # 진입 시간 [s] — 짧아야 "쑥" 들어온 것으로 보인다

# 무작위 배치 범위 — 배터리(y=-0.48)와 발전기(y=+0.5) 사이 통로.
RAND_Y = (-0.30, 0.10)
ARM_X = 0.33            # 손바닥 x — 운반 대역(0.27~0.46) 한가운데
ARM_Z = 0.30            # 손바닥 높이 — 운반 높이(≈0.31)와 같은 차단 높이

# 시드 1~5 → 손바닥 위치. x·z 는 같고 **y 만** 다르다.
ARM_PLACEMENTS = {
    1: (ARM_X, -0.30, ARM_Z),
    2: (ARM_X, -0.20, ARM_Z),
    3: (ARM_X, -0.10, ARM_Z),
    4: (ARM_X,  0.00, ARM_Z),
    5: (ARM_X,  0.10, ARM_Z),
}


class ArmIntruder:
    def __init__(self, obj, seed: int = 12345, placement: int = 0):
        """
        Args:
            placement: 1~10 이면 손바닥 위치를 ARM_PLACEMENTS 로 **고정**한다.
                0 이면 리셋마다 무작위로 뽑는다.
        """
        self.obj = obj
        self.placement = placement if placement in ARM_PLACEMENTS else 0
        self.rng = np.random.RandomState(seed if placement == 0 else placement)
        self.pattern = "y-0.20"
        self.hand = np.array([ARM_X, -0.20, ARM_Z])
        self.root = self.hand + np.array([_L, 0.0, 0.0])
        self.park = self.root.copy()
        self.state = "WAIT"
        self.t0 = time.monotonic()
        self._pick_spot()

    def _pick_spot(self) -> None:
        """이번 에피소드에 팔이 놓일 자리를 정한다."""
        if self.placement:
            self.hand = np.array(ARM_PLACEMENTS[self.placement], dtype=float)
        else:
            self.hand = np.array([ARM_X, self.rng.uniform(*RAND_Y), ARM_Z])
        self.pattern = f"y{self.hand[1]:+.2f}"
        # 수평 팔: 팔꿈치 원점 = 손바닥 + (L, 0, 0). 대기는 같은 y·z 에 x 만
        # 책상 밖이라, 진입이 순수한 x 직선 이동이 되어 "쑥" 이 살아난다.
        self.root = self.hand + np.array([_L, 0.0, 0.0])
        self.park = np.array([_PARK_X, self.root[1], self.root[2]])

    def reset(self) -> None:
        """에피소드 리셋 — 자리를 다시 뽑고 책상 밖에서 대기시킨다."""
        self._pick_spot()
        self.state = "WAIT"
        self.t0 = time.monotonic()
        self._write(self.park)

    @property
    def parked(self) -> bool:
        """아직 작업 영역 밖인가 — 충돌 판정을 걸지 말아야 하는 구간."""
        return self.state == "WAIT"

    @staticmethod
    def _smooth(t: float) -> float:
        return t * t * (3.0 - 2.0 * t)

    def _write(self, root) -> None:
        """수평 자세(항등 방향)로 위치만 쓴다."""
        base = self.obj.data.default_root_state.clone()
        base[:, 0] = float(root[0])
        base[:, 1] = float(root[1])
        base[:, 2] = float(root[2])
        base[:, 3] = 1.0        # quat w — 회전 없음(수평)
        base[:, 4:7] = 0.0
        self.obj.write_root_pose_to_sim(base[:, :7])
        self.obj.write_root_velocity_to_sim(torch.zeros_like(base[:, 7:]))

    def step(self, carrying: bool, interrupt: bool, ros, step: int) -> None:
        """대기 1초 → 직선 진입 → 그 자리 유지.

        carrying·interrupt 는 쓰지 않는다 — 진입 시점은 **초기화 후 경과 시간**
        하나로 정해진다. 호출부(runner)의 서명을 유지하려고 인자만 받는다.
        """
        el = time.monotonic() - self.t0
        if self.state == "WAIT":
            self._write(self.park)
            if el >= _ENTER_DELAY:
                self.state = "ENTER"
                self.t0 = time.monotonic()
                ros.event("arm_enter", step=step, pattern=self.pattern,
                          hand=[round(float(v), 3) for v in self.hand])
        elif self.state == "ENTER":
            s = self._smooth(min(el / _ENTER_T, 1.0))
            self._write(self.park + (self.root - self.park) * s)
            if el >= _ENTER_T:
                self.state = "HOLD"
        else:
            self._write(self.root)
