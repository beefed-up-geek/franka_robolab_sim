# SPDX-License-Identifier: Apache-2.0
"""task2 test — 작업자 팔 침입 상태기계 (runner 구동).

작업자는 절단된 책상 앞모서리(x=0.65) 밖에 서 있고, **팔꿈치부터 손까지만**
(사용자 지정) **앞뒤(x축)로 수평하게** 워크스페이스에 들어왔다 나간다.
팔 강체의 기하는 팔꿈치 원점에서 -X 로 뻗으므로(worker_arm.usda) 수평
자세 = 항등 방향이고, 파킹은 Y축 피치 -90°(팔 내림)다. 진입 시 팔꿈치
절단면(커프)이 책상 모서리 부근에 걸쳐 나머지 몸은 화면 밖으로 암시된다.
매 스텝 write_root_pose 재기록 — 자성 홀드와 같은 검증된 패턴이라 리셋
폭발이 없다. (킨매틱 플래그는 프레임워크 import_scene 이 RigidObject 로
등록하지 않아 못 쓴다 — 동적+재기록으로 해결.)

상태: IDLE → (운반 감지) ARMED → RAISE(모서리 회피 호로 수평까지) →
REACH(레인 높이로 하강) → DWELL(체류, 좌우 y 스윕 + 앞뒤 x 미동) →
PULL(호버 복귀) → LOWER(파킹) → COOL → IDLE.
양쪽 부착 완료·에피소드 리셋이면 즉시 후퇴한다.

레인 2종 — 난이도 축은 높이다 (train 배치: 배터리 (0.36,-0.48) z180,
단자 포스트 x 0.265·0.458 / y -0.43 / 상단 z 0.223, 운반 높이 z≈0.31):
  high: 손바닥 (0.32,-0.15,0.30) 부근 — 팔대가 x로 뻗어 운반 경로(두
        플러그→단자, x 0.27~0.46 대역)를 운반 높이에서 가로지른다. 실제 차단.
  low : 손바닥 (0.40, 0.06,0.20) 부근 — 운반 높이 아래·플러그 위(플러그
        상단 0.117 < 팔대 하단 0.154). 시각 압박 위주.
"""
from __future__ import annotations

import math
import time

import numpy as np
import torch

_L = 0.385                      # 팔꿈치 원점 -> 손바닥 중심 (손끝은 0.52)
# 파킹: 앞모서리 밖·상판 아래(z 0.10, 손끝 -0.42)·왼쪽 구석(y -0.45) — front
# 카메라 화면 중앙을 가리지 않도록 프레임 좌하단 밖으로 내렸다 (실측 프레임).
_PARK = np.array([0.78, -0.45, 0.10])
_HOVER_Z = 0.42                 # 진입·이탈 수평 호버 높이
_RAISE_T = 0.7
_REACH_T = 0.8
_PULL_T = 0.6
_LOWER_T = 0.6


class ArmIntruder:
    def __init__(self, obj, seed: int = 12345):
        self.obj = obj
        self.rng = np.random.RandomState(seed)
        self.state = "IDLE"
        self.t0 = time.monotonic()
        self.entries = 0
        self.max_entries = 2        # 드라이버 1회 실행(리셋 간) 최대 진입
        self.cur = (np.array(_PARK), math.radians(90.0))
        self.from_pose = self.cur
        self.hand = np.array([0.32, -0.15, 0.30])
        self.hover = np.array([0.9, -0.15, _HOVER_Z])
        self.target = self.hover.copy()
        self.pattern = "high"
        self.delay = 1.0
        self.dwell_dur = 3.0
        self.cool = 5.0
        self.sweep = 0.12

    def reset(self) -> None:
        self.state = "IDLE"
        self.entries = 0
        self.cur = (np.array(_PARK), math.radians(90.0))

    @staticmethod
    def _smooth(t: float) -> float:
        return t * t * (3.0 - 2.0 * t)

    def _arc(self, s: float):
        """파킹(팔 내림) <-> 수평 호버. 어깨를 +x(작업자 쪽)로 뺐다 오므려
        손이 책상 앞모서리를 크게 돌아 올라온다 (상판 관통 회피)."""
        root = np.array([
            _PARK[0] + (self.hover[0] - _PARK[0]) * s + 0.40 * math.sin(math.pi * s),
            _PARK[1] + (self.hover[1] - _PARK[1]) * s,
            _PARK[2] + (self.hover[2] - _PARK[2]) * s,
        ])
        return root, math.radians(90.0) * (1.0 - s)

    def _write(self, root, pitch: float) -> None:
        """pitch = 수평 아래로 처진 각. 기하가 -X 방향이라 Y축 -pitch 회전."""
        self.cur = (np.asarray(root, dtype=float).copy(), float(pitch))
        base = self.obj.data.default_root_state.clone()
        base[:, 0] = float(root[0])
        base[:, 1] = float(root[1])
        base[:, 2] = float(root[2])
        base[:, 3] = math.cos(pitch / 2.0)
        base[:, 4] = 0.0
        base[:, 5] = -math.sin(pitch / 2.0)
        base[:, 6] = 0.0
        self.obj.write_root_pose_to_sim(base[:, :7])
        self.obj.write_root_velocity_to_sim(torch.zeros_like(base[:, 7:]))

    def _sample_entry(self) -> None:
        if self.rng.rand() < 0.5:
            self.pattern = "high"
            self.hand = np.array([0.32 + self.rng.uniform(-0.04, 0.04),
                                  -0.15 + self.rng.uniform(-0.10, 0.10),
                                  0.30 + self.rng.uniform(-0.02, 0.02)])
        else:
            self.pattern = "low"
            self.hand = np.array([0.40 + self.rng.uniform(-0.05, 0.05),
                                  0.06 + self.rng.uniform(-0.08, 0.08),
                                  0.20 + self.rng.uniform(-0.02, 0.02)])
        # 수평 팔: 어깨 = 손바닥 + (L, 0, 0). 호버는 같은 xy, 높은 z.
        self.target = self.hand + np.array([_L, 0.0, 0.0])
        self.hover = np.array([self.target[0], self.target[1], _HOVER_Z])
        self.delay = float(self.rng.uniform(0.0, 2.0))
        self.dwell_dur = float(self.rng.uniform(2.0, 4.0))
        self.sweep = float(self.rng.choice([0.0, -0.12, 0.12]))

    def step(self, carrying: bool, interrupt: bool, ros, step: int) -> None:
        now = time.monotonic()
        if interrupt and self.state == "ARMED":
            self.state = "IDLE"
        if interrupt and self.state in ("RAISE", "REACH", "DWELL"):
            self.from_pose = self.cur
            self.state = "PULL"
            self.t0 = now
        el = now - self.t0

        st = self.state
        if st == "IDLE":
            self._write(*self._arc(0.0))
            if carrying and self.entries < self.max_entries:
                self._sample_entry()
                self.state = "ARMED"
                self.t0 = now
        elif st == "ARMED":
            self._write(*self._arc(0.0))
            if not carrying:
                self.state = "IDLE"
            elif el >= self.delay:
                self.state = "RAISE"
                self.t0 = now
                self.entries += 1
                ros.event("arm_enter", step=step, pattern=self.pattern,
                          hand=[round(float(v), 3) for v in self.hand])
        elif st == "RAISE":
            s = self._smooth(min(el / _RAISE_T, 1.0))
            self._write(*self._arc(s))
            if el >= _RAISE_T:
                self.state = "REACH"
                self.t0 = now
        elif st == "REACH":
            s = self._smooth(min(el / _REACH_T, 1.0))
            self._write(self.hover + (self.target - self.hover) * s, 0.0)
            if el >= _REACH_T:
                self.state = "DWELL"
                self.t0 = now
        elif st == "DWELL":
            ph = min(el / self.dwell_dur, 1.0)
            off = np.array([0.04 * math.sin(2.0 * math.pi * ph),
                            self.sweep * math.sin(math.pi * ph), 0.0])
            self._write(self.target + off, 0.0)
            if el >= self.dwell_dur:
                self.from_pose = self.cur
                self.state = "PULL"
                self.t0 = now
        elif st == "PULL":
            s = self._smooth(min(el / _PULL_T, 1.0))
            root = self.from_pose[0] + (self.hover - self.from_pose[0]) * s
            self._write(root, self.from_pose[1] * (1.0 - s))
            if el >= _PULL_T:
                self.state = "LOWER"
                self.t0 = now
        elif st == "LOWER":
            s = self._smooth(min(el / _LOWER_T, 1.0))
            self._write(*self._arc(1.0 - s))
            if el >= _LOWER_T:
                self.state = "COOL"
                self.t0 = now
                self.cool = float(self.rng.uniform(4.0, 8.0))
                ros.event("arm_exit", step=step)
        elif st == "COOL":
            self._write(*self._arc(0.0))
            if el >= self.cool:
                self.state = "IDLE"
