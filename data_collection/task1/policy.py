# SPDX-License-Identifier: Apache-2.0
"""task1 code-as-policy — 지정한 공구를 집어 노란 테이프 너머로 전달한다.

제어 코어(비례+속도감쇠, 도달 판정의 속도 조건, 눌렀다-올리는 파지 대기)는
task3 정책에서 검증된 것을 그대로 가져왔다 — 근거 주석은 task3/policy.py 에 있다.
여기서는 task1 에 맞게 세 가지가 다르다.

  1. 대상이 벨트 순서가 아니라 **인자로 지정**된다 (망치/드릴/가위 중 하나).
  2. 전달 자세 — 그리퍼는 수직 아래를 향한 채, 지정한 **요 각도**(0/90/180/270°
     등 임의각)로 돌려서 공구를 원하는 방향으로 내려놓는다. 회전은 운반 중에
     서보한다. 평행 그리퍼라 물리적으로는 180° 대칭이지만, 물린 공구의 머리
     방향이 다르므로 각도 그대로 존중한다.
  3. 전달 **속도**가 인자다 [m/s]. 명령 상한으로 변환된다:
     한 스텝 명령 = 속도 / (실현률 x 제어율).

전달 지점은 테이프(y=-0.40) 너머 작업자 구역 상공 (0.50, -0.52) — 내려놓으면
공구가 구역 안에 떨어진다. 경계를 넘은 공구는 시뮬레이션이 초기화한다(예정).
"""
from __future__ import annotations

import math

# ── task3 에서 검증된 제어 이득 ───────────────────────────────────────
GAIN = 2.5
DAMP = 4.0
SETTLE_VMAX = 0.012
MAX_STEP = 0.10
ROT_GAIN = 3.0
MAX_ROT = 0.25
MAX_ROT_CARRY = 0.12     # 운반 중 회전 상한 — 이동하면서 회전을 끝내야 하므로
                         # task3(0.05)보다 크다. CoM 파지점 고정으로 흔들림 없음.
POS_TOL = 0.007
COARSE_TOL = 0.02
SETTLE_STEPS = 3
ESCAPE_H = 0.12
REALIZED = 0.30
RATE_HZ = 6.0

STAGE_MAX_STEP = {
    "TRANSIT": 0.18,
    "APPROACH": 0.08,
    "DESCEND": 0.06,
    "LIFT": 0.05,
    # DELIVER 는 인자로 정해진다 (아래 __init__)
}

# 수직 파지 자세 (task3 과 동일)
VERTICAL_QUAT = (0.7071068, 0.0, 0.7071068, 0.0)

# ── task1 기하 ───────────────────────────────────────────────────────
SURFACE_Z = 0.003        # 워크스페이스 매트 윗면 (상판 0 + 3mm)
MIN_TCP_CLEAR = 0.004
GRASP_DEPTH = 0.008
APPROACH_H = 0.10
TRANSIT_Z = 0.45         # 플랜지 기준 이동 고도
CARRY_TCP_Z = 0.34       # 운반 손끝 고도. 0.28 에서는 테이프 부근에서 든 공구가
                         # 눌리며(접촉 38N 스파이크) 빠졌다 — 더 높이 지나간다
# 전달 = 내려놓기가 아니라 **선 넘기기**다. 공구가 경계를 넘는 순간 시뮬레이션이
# 자동으로 초기화하므로(runner), 운반 고도를 유지한 채 이 지점까지 쭉 간다.
CROSS_XY = (0.48, -0.58)
TAPE_Y = -0.40

# 파지점 오프셋 [m, 월드 xy] — 원점이 파지 적합점과 다른 공구용.
# 드릴은 L자형이라 원점을 노리면 손가락이 넓어지는 몸통에 걸린다. 정점 슬라이스
# 측정(원점 상대): 몸통이 dx>+2(y폭 0.09~0.18)와 dx<-6(0.10), 손잡이는
# dx -5..-1 에서 y폭 0.045(축 y≈-0.035) — 그 좁은 구간의 정중앙을 잡는다.
# 굽힌 리셋 자세(손잡이 X축 평행)가 고정이라 월드 상수로 충분하다.
GRASP_OFFSET = {
    "cordless_drill": (-0.03, -0.035),
}

GRIP_WAIT = 18
CLOSE_PRESS_MAX = 0.006
CLOSE_PRESS_STEPS = 2
CLOSE_RISE = 0.004
CLOSE_END_Z = CLOSE_RISE * (GRIP_WAIT - 1 - CLOSE_PRESS_STEPS) - CLOSE_PRESS_MAX
LIFT_H = 0.05
OPEN_WAIT = 4
GRIP_LOST_STEPS = 5
ROT_ALIGN_TOL = 0.10     # 놓기 전 요 정렬 허용 오차 [rad]


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


def _qz(deg):
    r = math.radians(deg) / 2.0
    return (math.cos(r), 0.0, 0.0, math.sin(r))


def rot_error(current, target):
    """current → target 회전을 월드 회전 벡터로 (최단 경로)."""
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


class Task1DeliverPolicy:
    """지정 공구를 집어 테이프 너머로 전달한다.

    Args:
        tool: 씬 프림 이름 (hammer_7 | cordless_drill | scissors).
        yaw_deg: 전달 요 각도 [deg]. 그리퍼가 수직 아래를 향한 채 이만큼 돌려
            내려놓는다. 0 이면 집을 때 자세 그대로다.
        speed: 전달(운반) 속도 [m/s]. DELIVER 단계의 명령 상한으로 변환된다.
    """

    def __init__(self, tool: str, yaw_deg: float = 0.0, speed: float = 0.2) -> None:
        self.tool = tool
        self.yaw_deg = float(yaw_deg) % 360.0
        self.speed = max(0.03, min(0.6, float(speed)))
        # 한 스텝 명령 = 속도 / (실현률 x 제어율)
        self.deliver_step = self.speed / (REALIZED * RATE_HZ)
        self.deliver_quat = _qmul(_qz(self.yaw_deg), VERTICAL_QUAT)
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

    # ── 내부 ────────────────────────────────────────────────────────
    def _grasp_flange_z(self, tool: dict, flange_offset: float) -> float:
        """파지 플랜지 높이 — 손끝을 공구 중심 아래 8mm, 매트 위 4mm 는 남긴다."""
        center_z = tool["pos"][2]
        tcp = max(center_z - GRASP_DEPTH, SURFACE_Z + MIN_TCP_CLEAR)
        return tcp + flange_offset

    def _grasp_xy(self, tool: dict):
        ox, oy = GRASP_OFFSET.get(tool["name"], (0.0, 0.0))
        return tool["flange"][0] + ox, tool["flange"][1] + oy

    def _goal(self, tool: dict, eef, flange_offset: float):
        fx, fy = self._grasp_xy(tool)
        f = tool["flange"]
        if self.stage == "TRANSIT":
            return [fx, fy, TRANSIT_Z]
        if self.stage == "APPROACH":
            return [fx, fy, f[2] + APPROACH_H]
        if self.stage == "DESCEND":
            return [fx, fy, self._grasp_flange_z(tool, flange_offset)]
        if self.stage in ("CLOSE", "LIFT"):
            return self.locked_goal
        if self.stage in ("DELIVER", "OPEN"):
            # 수평 유지 — 내려가지 않는다. 선을 넘으면 심이 알아서 초기화한다.
            return [CROSS_XY[0], CROSS_XY[1], CARRY_TCP_Z + flange_offset]
        return [CROSS_XY[0], CROSS_XY[1], CARRY_TCP_Z + flange_offset + 0.1]  # CLEAR

    def act(self, eef, eef_quat, tools: list[dict], flange_offset: float,
            gripping: bool = False):
        """(delta6, gripper_close, info). eef/tools 는 ROS 관측 그대로."""
        info = {"stage": self.stage, "target": self.tool,
                "yaw": self.yaw_deg, "speed": self.speed}
        vel = ([e - p for e, p in zip(eef, self.prev_eef)]
               if self.prev_eef is not None else [0.0, 0.0, 0.0])
        self.prev_eef = list(eef)
        speed_now = math.sqrt(sum(v * v for v in vel))

        carrying = self.stage in ("CLOSE", "LIFT", "DELIVER", "OPEN")
        # 파지 전에는 수직 자세. 들어올리는 순간부터 '수직 아래 + 전달 요' 를 향해
        # 서보한다 — deliver_quat 자체가 수직 자세의 z 회전이라, 오차 벡터에 기울기
        # 성분이 섞이면 즉시 0 으로 눌리고 요 성분만 남는다. 제자리 회전 단계 없이
        # 이동(LIFT→DELIVER) 중에 회전을 끝낸다. 드릴은 CoM 을 파지점에 고정해
        # 두어 이동+회전 동시 동작에도 흔들리지 않는다.
        rot_target = self.deliver_quat \
            if self.stage in ("LIFT", "DELIVER", "OPEN", "CLEAR") \
            else VERTICAL_QUAT
        if eef_quat:
            rot_err_v = rot_error(eef_quat, rot_target)
            if self.stage in ("LIFT", "DELIVER"):
                ang = math.sqrt(sum(v * v for v in rot_err_v))
                if ang > 2.85:
                    # 요 180° 부근은 최단경로가 둘이라 스텝마다 축이 뒤집혀
                    # 제자리 떨림으로 멈춘다 — +z 로 밀어 대칭을 깬다.
                    rot_err_v = [0.0, 0.0, ang]
            rot = _clamp([v * ROT_GAIN for v in rot_err_v],
                         MAX_ROT_CARRY if carrying else MAX_ROT)
        else:
            rot = [0, 0, 0]

        if self.stage == "SEARCH":
            if any(t["name"] == self.tool for t in tools):
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

        tool = next((t for t in tools if t["name"] == self.tool), None)
        if tool is None:
            self.stage = "ESCAPE"
            self.escape_goal = [eef[0], eef[1], eef[2] + ESCAPE_H]
            self.fail_why = "목표 공구가 관측에서 사라짐"
            return [0.0] * 6, False, {**info, "stage": "ESCAPE"}

        grip = self.stage in ("CLOSE", "LIFT", "DELIVER")
        if self.stage in ("LIFT", "DELIVER"):
            self.no_grip = 0 if gripping else self.no_grip + 1
            if self.no_grip >= GRIP_LOST_STEPS:
                self.escape_goal = [eef[0], eef[1], eef[2] + ESCAPE_H]
                self.fail_why = "운반 중 놓침"
                self.stage = "ESCAPE"
                return [0.0] * 6, False, {**info, "stage": "ESCAPE",
                                          "why": self.fail_why}

        # CLOSE/OPEN — 대기하되 가만히 있지 않는다 (task3 press-rise)
        if self.stage in ("CLOSE", "OPEN"):
            self.wait += 1
            limit = GRIP_WAIT if self.stage == "CLOSE" else OPEN_WAIT
            if self.wait >= limit:
                self.wait = 0
                if self.stage == "CLOSE":
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
                else:
                    self.stage = "CLEAR"
                info["stage"] = self.stage
            gg = list(self.locked_goal or eef)
            if self.stage == "CLOSE":
                if self.wait <= CLOSE_PRESS_STEPS:
                    gg[2] -= CLOSE_PRESS_MAX * self.wait / CLOSE_PRESS_STEPS
                else:
                    gg[2] -= CLOSE_PRESS_MAX
                    gg[2] += CLOSE_RISE * (self.wait - CLOSE_PRESS_STEPS)
            d3 = _clamp([(g - e) * GAIN - DAMP * v
                         for g, e, v in zip(gg, eef, vel)], MAX_STEP)
            return [*d3, *rot], self.stage in ("CLOSE", "LIFT", "DELIVER"), info

        goal = self._goal(tool, eef, flange_offset)
        err_vec = [g - e for g, e in zip(goal, eef)]
        err = math.sqrt(sum(v * v for v in err_vec))
        info["err"] = round(err, 4)
        limit = self.deliver_step if self.stage == "DELIVER" \
            else STAGE_MAX_STEP.get(self.stage, MAX_STEP)
        delta3 = _clamp([ev * GAIN - DAMP * v for ev, v in zip(err_vec, vel)], limit)

        tol = POS_TOL if self.stage in ("APPROACH", "DESCEND") else COARSE_TOL
        if err < tol and speed_now < SETTLE_VMAX:
            self.hold += 1
            if self.hold >= SETTLE_STEPS:
                self.hold = 0
                if self.stage == "TRANSIT":
                    self.stage = "APPROACH"
                elif self.stage == "APPROACH":
                    self.stage = "DESCEND"
                elif self.stage == "DESCEND":
                    fx, fy = self._grasp_xy(tool)
                    self.locked_goal = [fx, fy,
                                        self._grasp_flange_z(tool, flange_offset)]
                    self.stage = "CLOSE"
                elif self.stage == "LIFT":
                    if tool["pos"][2] < SURFACE_Z + 0.03:
                        self.escape_goal = [eef[0], eef[1], eef[2] + ESCAPE_H]
                        self.fail_why = "들리지 않음"
                        self.stage = "ESCAPE"
                        return [0.0] * 6, False, {**info, "stage": "ESCAPE"}
                    self.stage = "DELIVER"
                elif self.stage == "DELIVER":
                    # 목표점까지 갔는데도 심의 경계 초기화가 안 왔다면 공구가
                    # 이미 빠졌거나 판정이 어긋난 것 — 드라이버 타임아웃에 맡긴다.
                    pass
                elif self.stage == "CLEAR":
                    self.reset()
                    return [0.0] * 6, False, {**info, "done": True}
                info["stage"] = self.stage
        else:
            self.hold = 0

        return [*delta3, *rot], grip, info
