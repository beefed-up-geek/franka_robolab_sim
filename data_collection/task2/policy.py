# SPDX-License-Identifier: Apache-2.0
"""task2 code-as-policy — 커넥터를 집어 배터리 단자 위에 씌운다.

제어 코어(비례+속도감쇠·도달 판정·눌렀다-올리는 파지 대기)는 task1 정책에서
검증된 것을 그대로 가져왔다. task2 에 맞게 다른 점:

  1. 파지 대상은 커넥터의 T-그립 (X축 평행, 커넥터 원점 위 56mm).
  2. 목표는 위치 인자로 받은 **배터리 단자 상공** — 내려놓기가 아니라
     하강 중 시뮬레이션의 부착 판정(connector_attached)이 끝을 낸다.
     드라이버가 이벤트를 보고 루프를 끊으므로 정책의 PLACE 는 계속 내려간다.
"""
from __future__ import annotations

import math

GAIN = 2.5
DAMP = 4.0
SETTLE_VMAX = 0.012
MAX_STEP = 0.10
ROT_GAIN = 3.0
MAX_ROT = 0.25
MAX_ROT_CARRY = 0.08
POS_TOL = 0.007
COARSE_TOL = 0.02
SETTLE_STEPS = 3
ESCAPE_H = 0.12

STAGE_MAX_STEP = {
    "TRANSIT": 0.18,
    "APPROACH": 0.08,
    "DESCEND": 0.06,
    "LIFT": 0.05,
    "MOVE": 0.06,
    "ALIGN": 0.05,
    "PLACE": 0.02,           # 단자 위 하강은 이주 천천히 — 약한 파지를 지킨다
}

VERTICAL_QUAT = (0.7071068, 0.0, 0.7071068, 0.0)

GRIP_Z_OFF = 0.087           # 커넥터 원점 → 패들 그립 중심
SURFACE_Z = 0.0005
APPROACH_H = 0.10
TRANSIT_Z = 0.45
CARRY_TCP_Z = 0.40           # 충분히 높게 들고 옮긴다 (요청) — 배터리+기둥 위 18cm
PLACE_STOP_MARGIN = -0.018   # 소켓이 포스트를 타고 내려가 씌워질 때까지 하강

GRIP_WAIT = 18
CLOSE_PRESS_MAX = 0.005
CLOSE_PRESS_STEPS = 2
CLOSE_RISE = 0.004
CLOSE_END_Z = CLOSE_RISE * (GRIP_WAIT - 1 - CLOSE_PRESS_STEPS) - CLOSE_PRESS_MAX
LIFT_H = 0.12
GRIP_LOST_STEPS = 10


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


def rot_error(current, target):
    inv = (current[0], -current[1], -current[2], -current[3])
    w, x, y, z = _qmul(target, inv)
    s = math.sqrt(x*x + y*y + z*z)
    if s < 1e-9:
        return [0.0, 0.0, 0.0]
    ang = 2.0 * math.atan2(s, w)
    if ang > math.pi:
        ang -= 2.0 * math.pi
    return [x / s * ang, y / s * ang, z / s * ang]


def _clamp(vec, limit):
    n = math.sqrt(sum(v * v for v in vec))
    if n <= limit or n == 0.0:
        return list(vec)
    k = limit / n
    return [v * k for v in vec]


class Task2ConnectPolicy:
    """커넥터 하나를 집어 지정 단자 위에 씌운다.

    Args:
        connector: 프림 이름 (connector_red | connector_black).
        terminal: 단자 포스트 상단의 월드 좌표 (x, y, z).
    """

    def __init__(self, connector: str, terminal) -> None:
        self.connector = connector
        self.terminal = list(terminal)
        self.prev_eef = None
        self.reset()

    def reset(self) -> None:
        self.stage = "SEARCH"
        self.hold = 0
        self.wait = 0
        self.locked_goal = None
        self.escape_goal = None
        self.fail_why = None
        self.no_grip = 0

    def _grasp_flange_z(self, tool, flange_offset):
        return tool["pos"][2] + GRIP_Z_OFF + flange_offset

    def _goal(self, tool, flange_offset, eef=None):
        p = tool["pos"]
        if self.stage == "TRANSIT":
            return [p[0], p[1], TRANSIT_Z]
        if self.stage == "APPROACH":
            return [p[0], p[1], self._grasp_flange_z(tool, flange_offset) + APPROACH_H]
        if self.stage == "DESCEND":
            return [p[0], p[1], self._grasp_flange_z(tool, flange_offset)]
        if self.stage in ("CLOSE", "LIFT"):
            return self.locked_goal
        if self.stage == "MOVE":
            # eef 기준 이동 — 커넥터 폐루프를 이동 중에 쓰면 흔들림과 양성
            # 되먹임을 일으켜 떨어뜨린다 (실측). 정렬은 ALIGN 에서 한다.
            return [self.terminal[0], self.terminal[1], CARRY_TCP_Z + flange_offset]
        if self.stage == "ALIGN":
            # 포스트 위 3.5cm 호버에서 커넥터 관측 기준으로 xy 를 수 mm 까지
            # 맞춘다 — 접촉 없는 높이라 반력 싸움이 없다. 그 뒤 수직 삽입.
            gx = eef[0] + (self.terminal[0] - p[0])
            gy = eef[1] + (self.terminal[1] - p[1])
            gz = eef[2] + ((self.terminal[2] + 0.035) - p[2])
            return [gx, gy, gz]
        # PLACE — 커넥터 관측 기준 폐루프. xy 는 그대로, z 는 절반 이득으로
        # 감쇠해 살살 내려간다 (급강하가 약한 파지를 떨어뜨린 실측 교훈).
        gx = eef[0] + (self.terminal[0] - p[0])
        gy = eef[1] + (self.terminal[1] - p[1])
        gz = eef[2] + 0.5 * ((self.terminal[2] + PLACE_STOP_MARGIN) - p[2])
        return [gx, gy, gz]

    def act(self, eef, eef_quat, tools, flange_offset, gripping=False):
        info = {"stage": self.stage, "target": self.connector}
        vel = ([e - p for e, p in zip(eef, self.prev_eef)]
               if self.prev_eef is not None else [0.0, 0.0, 0.0])
        self.prev_eef = list(eef)
        speed_now = math.sqrt(sum(v * v for v in vel))

        carrying = self.stage in ("CLOSE", "LIFT", "MOVE", "ALIGN", "PLACE")
        rot = _clamp([v * ROT_GAIN for v in rot_error(eef_quat, VERTICAL_QUAT)],
                     MAX_ROT_CARRY if carrying else MAX_ROT) if eef_quat else [0, 0, 0]

        if self.stage == "SEARCH":
            if any(t["name"] == self.connector for t in tools):
                self.stage = "TRANSIT"
                info["stage"] = self.stage
            return [0, 0, 0, *rot], False, info

        if self.stage == "ESCAPE":
            d3 = _clamp([(g - e) * GAIN - DAMP * v
                         for g, e, v in zip(self.escape_goal, eef, vel)], MAX_STEP)
            if math.dist(self.escape_goal, eef) < 0.03:
                why = self.fail_why
                self.reset()
                return [0.0] * 6, False, {**info, "stage": "SEARCH",
                                          "abort": True, "why": why}
            return [*d3, *rot], False, info

        tool = next((t for t in tools if t["name"] == self.connector), None)
        if tool is None:
            self.stage = "ESCAPE"
            self.escape_goal = [eef[0], eef[1], eef[2] + ESCAPE_H]
            self.fail_why = "커넥터가 관측에서 사라짐"
            return [0.0] * 6, False, {**info, "stage": "ESCAPE"}

        grip = self.stage in ("CLOSE", "LIFT", "MOVE", "ALIGN", "PLACE")
        if self.stage in ("LIFT", "MOVE", "ALIGN", "PLACE"):
            self.no_grip = 0 if gripping else self.no_grip + 1
            if self.no_grip >= GRIP_LOST_STEPS:
                self.escape_goal = [eef[0], eef[1], eef[2] + ESCAPE_H]
                self.fail_why = "운반 중 놓침"
                self.stage = "ESCAPE"
                return [0.0] * 6, False, {**info, "stage": "ESCAPE",
                                          "why": self.fail_why}

        if self.stage == "CLOSE":
            self.wait += 1
            if self.wait >= GRIP_WAIT:
                self.wait = 0
                if not gripping:
                    self.no_grip += 1
                    if self.no_grip < GRIP_LOST_STEPS:
                        self.wait = GRIP_WAIT - 1
                        return [0.0, 0.0, 0.0, *rot], True, info
                    self.escape_goal = [eef[0], eef[1], eef[2] + ESCAPE_H]
                    self.fail_why = "파지 실패"
                    self.stage = "ESCAPE"
                    return [0.0] * 6, False, {**info, "stage": "ESCAPE"}
                self.no_grip = 0
                self.locked_goal = [self.locked_goal[0], self.locked_goal[1],
                                    self.locked_goal[2] + CLOSE_END_Z + LIFT_H]
                self.stage = "LIFT"
                info["stage"] = self.stage
            gg = list(self.locked_goal)
            if self.wait <= CLOSE_PRESS_STEPS:
                gg[2] -= CLOSE_PRESS_MAX * self.wait / CLOSE_PRESS_STEPS
            else:
                gg[2] -= CLOSE_PRESS_MAX
                gg[2] += CLOSE_RISE * (self.wait - CLOSE_PRESS_STEPS)
            d3 = _clamp([(g - e) * GAIN - DAMP * v
                         for g, e, v in zip(gg, eef, vel)], MAX_STEP)
            return [*d3, *rot], True, info

        goal = self._goal(tool, flange_offset, eef)
        err_vec = [g - e for g, e in zip(goal, eef)]
        err = math.sqrt(sum(v * v for v in err_vec))
        info["err"] = round(err, 4)
        limit = STAGE_MAX_STEP.get(self.stage, MAX_STEP)
        delta3 = _clamp([ev * GAIN - DAMP * v for ev, v in zip(err_vec, vel)], limit)

        tol = POS_TOL if self.stage in ("APPROACH", "DESCEND", "MOVE") else COARSE_TOL
        if self.stage == "ALIGN":
            tol = 0.004
        if err < tol and speed_now < SETTLE_VMAX:
            self.hold += 1
            if self.hold >= SETTLE_STEPS:
                self.hold = 0
                if self.stage == "TRANSIT":
                    self.stage = "APPROACH"
                elif self.stage == "APPROACH":
                    self.stage = "DESCEND"
                elif self.stage == "DESCEND":
                    p = tool["pos"]
                    self.locked_goal = [p[0], p[1],
                                        self._grasp_flange_z(tool, flange_offset)]
                    self.stage = "CLOSE"
                elif self.stage == "LIFT":
                    if tool["pos"][2] < SURFACE_Z + 0.03:
                        self.escape_goal = [eef[0], eef[1], eef[2] + ESCAPE_H]
                        self.fail_why = "들리지 않음"
                        self.stage = "ESCAPE"
                        return [0.0] * 6, False, {**info, "stage": "ESCAPE"}
                    self.stage = "MOVE"
                elif self.stage == "MOVE":
                    self.stage = "ALIGN"
                elif self.stage == "ALIGN":
                    self.stage = "PLACE"
                elif self.stage == "PLACE":
                    # 목표까지 내려갔는데 부착 이벤트가 안 왔다 — 드라이버 타임아웃에
                    # 맡긴다 (판정 반경 밖일 수 있다).
                    pass
                info["stage"] = self.stage
        else:
            self.hold = 0

        return [*delta3, *rot], grip, info
