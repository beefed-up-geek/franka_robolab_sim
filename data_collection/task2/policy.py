# SPDX-License-Identifier: Apache-2.0
"""task2 code-as-policy v2 — 다양한 궤적·속도의 목표 지향 시연.

VLA 를 test-time steering 하려면 상태마다 다른 방향의 액션이 학습 분포에
있어야 하고, 동시에 수렴하려면 모든 시연이 목표로 전진해야 한다 (사용자
설계 원칙). 다양성 축은 세 개로 좁혔다 (사용자 지정):

  · 위아래(z) — 저공/고공·아치(깊이/위치 변주)·이중 아치·골짜기·계단·
    하강 출발·큰 물결·대각 상승. xy 는 항상 목표 직선 위라 ±z 성분만
    상태별로 위/아래 양방향 분포를 만든다 (사용자 지정: ±y 커브 불필요)
  · 이동 속도 — 에피소드 배수 1.0~1.6x

앞뒤(경로 진행 방향) 가감속과 좌우(±y) 변주는 두지 않는다. 두 불변식:

  1. 목표까지의 xy 거리가 웨이포인트마다 단조 감소 (_monotone 이 강제) —
     멀어지는 구간이 없어 같은 상태에서 상반된 감독 신호가 없다.
  2. **배터리 근처(진행 ~70% 이후)는 방향 변주 없이 단자로 곧장 수렴** —
     모든 가족의 마지막 구간은 호버 지점을 향한 직선이고, 요동도
     목표 20cm 안에서는 꺼진다.

파지(SEARCH~LIFT)와 정밀 구간(ALIGN·PLACE)은 검증된 폐루프 그대로,
운반(MOVE)만 가족·속도를 샘플링한다. 기본 스텝 상한은 v1 대비 30~60%
올렸다 (지금보다 빠르게 — 사용자 요청).
"""
from __future__ import annotations

import math
import random

GAIN = 2.5
DAMP = 4.0
SETTLE_VMAX = 0.012
MAX_STEP = 0.10
ROT_GAIN = 3.0
MAX_ROT = 0.25
MAX_ROT_CARRY = 0.08
POS_TOL = 0.007
COARSE_TOL = 0.02
SETTLE_STEPS = 2
ESCAPE_H = 0.12

# v1 대비 상향한 기본 스텝 상한 — 에피소드 속도 배수가 다시 곱해진다.
# ALIGN·PLACE 는 삽입 정밀이 걸려 있어 배수를 1.15 로 제한한다.
BASE_STEP = {
    "TRANSIT": 0.24,
    "APPROACH": 0.12,
    "DESCEND": 0.09,
    "LIFT": 0.09,
    "MOVE": 0.13,
    "ALIGN": 0.06,
    "PLACE": 0.026,
}

VERTICAL_QUAT = (0.7071068, 0.0, 0.7071068, 0.0)

GRIP_Z_OFF = 0.087           # 커넥터 원점 → 패들 그립 중심
SURFACE_Z = 0.0005
PLACE_STOP_MARGIN = -0.018   # 소켓이 포스트를 타고 내려가 씌워질 때까지

CLOSE_PRESS_MAX = 0.005
CLOSE_PRESS_STEPS = 2
CLOSE_RISE = 0.004
GRIP_LOST_STEPS = 10

# 운반 웨이포인트 안전 상자 (플랜지 좌표) — IK 도달 반경·발전기(y>0.35)
# 회피·포스트 상공 여유(플랜지 0.48 = 커넥터 바닥 0.25, 포스트 0.223).
WP_X = (0.18, 0.60)
WP_Y = (-0.58, 0.30)
WP_Z = (0.42, 0.66)
WP_R3 = 0.80

FAMILIES = {
    0: "직선 저공", 1: "직선 고공", 2: "아치", 3: "대각 상승",
    4: "이중 아치", 5: "깊은 아치", 6: "골짜기", 7: "계단",
    8: "하강 출발", 9: "큰 물결", 10: "조기 아치", 11: "후기 아치",
}


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
    """커넥터 하나를 집어 지정 단자 위에 씌운다 (궤적·속도 샘플링).

    Args:
        connector: 프림 이름 (connector_red | connector_black).
        terminal: 단자 포스트 상단의 월드 좌표 (x, y, z).
        rng: 에피소드 난수원 (재현성은 드라이버가 시드로 보장).
        speed: 속도 배수 고정값 (없으면 0.9~1.5 샘플).
        family: 궤적 가족 고정값 (없으면 14개 중 샘플).
    """

    def __init__(self, connector: str, terminal, rng=None,
                 speed=None, family=None) -> None:
        self.connector = connector
        self.terminal = list(terminal)
        self.rng = rng if rng is not None else random.Random()
        self.speed = float(speed) if speed else self.rng.uniform(1.0, 1.6)
        self.family = (int(family) if family is not None
                       else self.rng.randrange(len(FAMILIES)))
        prec = min(self.speed, 1.15)
        self.caps = {k: v * (prec if k in ("ALIGN", "PLACE") else self.speed)
                     for k, v in BASE_STEP.items()}
        self.grip_wait = max(10, round(16 / self.speed))
        self.close_end_z = (CLOSE_RISE * (self.grip_wait - 1 - CLOSE_PRESS_STEPS)
                            - CLOSE_PRESS_MAX)
        self.transit_z = self.rng.uniform(0.42, 0.50)
        self.approach_h = self.rng.uniform(0.07, 0.12)
        # 하강 출발(F8)은 높이 들었다 내려가고, 대각 상승(F3)은 낮게 들어
        # 이동하며 올라간다 — 시작 z 부터 다양해진다.
        self.lift_h = (0.06 if self.family == 3
                       else self.rng.uniform(0.16, 0.20) if self.family == 8
                       else 0.12)
        self.hover_z = {0: 0.50, 1: 0.60}.get(self.family,
                                              self.rng.uniform(0.50, 0.58))
        # 가는 길의 z 물결 — 다양하고 크게 (사용자 요청): 진폭 기본 ±2~6cm
        # (큰 물결 가족은 ±6~10cm), 주기는 느린 너울~빠른 잔물결(4배 폭),
        # 위상 무작위, 50% 는 2화음을 얹어 비정형 물결을 만든다.
        # 배터리 20cm 안에서는 꺼지고, z 안전상자(0.42~0.66)로 클램프된다.
        self.z_amp = (self.rng.uniform(0.06, 0.10) if self.family == 9
                      else self.rng.uniform(0.02, 0.06))
        self.z_freq = self.rng.uniform(0.15, 0.60)
        self.z_phase = self.rng.uniform(0.0, 6.283)
        if self.rng.random() < 0.5:
            self.z_amp2 = self.z_amp * self.rng.uniform(0.3, 0.6)
            self.z_freq2 = self.z_freq * self.rng.uniform(1.7, 2.6)
            self.z_phase2 = self.rng.uniform(0.0, 6.283)
        else:
            self.z_amp2, self.z_freq2, self.z_phase2 = 0.0, 1.0, 0.0
        # 일시 감속·정지 이벤트 (사용자 요청) — 운반 중 0~2회, 4~10스텝
        # (0.7~1.7초) 동안 속도 상한을 0~30% 로 낮춘다 (0 이면 잠깐 정지).
        # 정지는 '멈춤'이라 목표 접근 단조성을 깨지 않고, 배터리 20cm
        # 안에서는 발동하지 않는다.
        self.pauses = []
        t = 8
        for _ in range(self.rng.choice([0, 1, 1, 2])):
            t0 = t + self.rng.randint(0, 14)
            dur = self.rng.randint(4, 10)
            self.pauses.append((t0, t0 + dur, self.rng.uniform(0.0, 0.3)))
            t = t0 + dur + 8
        self.prev_eef = None
        self.reset()

    def describe(self) -> str:
        pz = f" 정지{len(self.pauses)}회" if self.pauses else ""
        harm = "2화음" if self.z_amp2 else "단일"
        return (f"{FAMILIES[self.family]}(F{self.family}){pz} "
                f"물결±{self.z_amp*100:.0f}cm/{harm} 속도 x{self.speed:.2f}")

    def reset(self) -> None:
        self.stage = "SEARCH"
        self.hold = 0
        self.wait = 0
        self.locked_goal = None
        self.escape_goal = None
        self.fail_why = None
        self.no_grip = 0
        self.route = []
        self.wp_i = 0
        self.move_tick = 0
        self.total_xy = 1.0
        self.perp = (0.0, 1.0)

    # ── 운반 경로 생성 ────────────────────────────────────────────────
    def _fit(self, p):
        x = min(max(p[0], WP_X[0]), WP_X[1])
        y = min(max(p[1], WP_Y[0]), WP_Y[1])
        z = min(max(p[2], WP_Z[0]), WP_Z[1])
        if x*x + y*y + z*z > WP_R3*WP_R3:
            z2 = WP_R3*WP_R3 - x*x - y*y
            z = max(WP_Z[0], math.sqrt(z2)) if z2 > WP_Z[0]**2 else WP_Z[0]
        return [x, y, z]

    def _monotone(self, S, E, route):
        """불변식 1: 목표까지 xy 거리가 웨이포인트마다 단조 감소."""
        out = []
        d_prev = math.dist(S[:2], E[:2])
        for p in route[:-1]:
            d = math.dist(p[:2], E[:2])
            if d > d_prev - 0.01:
                k = max(0.0, (d_prev - 0.02) / max(d, 1e-6))
                p = [E[0] + (p[0] - E[0]) * k,
                     E[1] + (p[1] - E[1]) * k, p[2]]
                d = math.dist(p[:2], E[:2])
            out.append(p)
            d_prev = d
        out.append(route[-1])
        return out

    def _build_route(self, S):
        f, rng = self.family, self.rng
        E = [self.terminal[0], self.terminal[1], self.hover_z]
        ux, uy = E[0] - S[0], E[1] - S[1]
        n = math.hypot(ux, uy) or 1e-6
        ux, uy = ux / n, uy / n
        self.perp = (-uy, ux)
        self.total_xy = n

        def mid(a):
            return [S[0] + (E[0]-S[0])*a, S[1] + (E[1]-S[1])*a,
                    S[2] + (E[2]-S[2])*a]

        def zpt(a, dz):
            q = mid(a)
            q[2] = min(max(q[2] + dz, WP_Z[0]), WP_Z[1])
            return q

        zmax = max(S[2], E[2])
        # xy 는 전부 목표 직선 위 — z 프로파일만 가족별로 다르다.
        # 배터리 근처(마지막 구간)는 호버로 자연 수렴한다.
        if f == 2:                                   # 아치
            route = [zpt(0.5, zmax - mid(0.5)[2] + rng.uniform(0.06, 0.10)), E]
        elif f == 4:                                 # 이중 아치 — 상승·하강 2회
            up1, dip, up2 = (rng.uniform(0.05, 0.08), rng.uniform(0.02, 0.05),
                             rng.uniform(0.04, 0.07))
            route = [zpt(0.28, up1), zpt(0.5, -dip), zpt(0.72, up2), E]
        elif f == 5:                                 # 깊은 아치 — 큰 상승
            route = [zpt(0.5, zmax - mid(0.5)[2] + rng.uniform(0.11, 0.15)), E]
        elif f == 6:                                 # 골짜기 — 내려갔다 올라온다
            route = [zpt(0.5, -(mid(0.5)[2] - WP_Z[0]) * rng.uniform(0.6, 1.0)), E]
        elif f == 7:                                 # 계단 — 상승·수평·하강
            zhi = min(WP_Z[1], E[2] + rng.uniform(0.07, 0.12))
            p1, p2 = mid(0.15), mid(0.85)
            p1[2] = p2[2] = zhi
            route = [p1, p2, E]
        elif f == 8:                                 # 하강 출발 — 높게 들고 내려가며 이동
            route = [zpt(0.45, -rng.uniform(0.05, 0.09)), E]
        elif f == 10:                                # 조기 아치 — 봉우리가 앞쪽
            route = [zpt(0.3, rng.uniform(0.08, 0.12)), E]
        elif f == 11:                                # 후기 아치 — 봉우리가 뒤쪽
            route = [zpt(0.65, rng.uniform(0.08, 0.12)), E]
        else:                                        # 0,1,3,9 — 직선 계열(+물결)
            route = [E]

        self.route = self._monotone(S, [E[0], E[1], E[2]],
                                    [self._fit(p) for p in route])
        self.wp_i = 0
        self.move_tick = 0

    # ── 단계별 목표 ───────────────────────────────────────────────────
    def _grasp_flange_z(self, tool, flange_offset):
        return tool["pos"][2] + GRIP_Z_OFF + flange_offset

    def _goal(self, tool, flange_offset, eef=None):
        p = tool["pos"]
        if self.stage == "TRANSIT":
            return [p[0], p[1], self.transit_z]
        if self.stage == "APPROACH":
            return [p[0], p[1], self._grasp_flange_z(tool, flange_offset) + self.approach_h]
        if self.stage == "DESCEND":
            return [p[0], p[1], self._grasp_flange_z(tool, flange_offset)]
        if self.stage in ("CLOSE", "LIFT"):
            return self.locked_goal
        if self.stage == "MOVE":
            g = list(self.route[self.wp_i])
            # z 물결은 목표에서 20cm 밖에서만 — 배터리 근처는 곧장 수렴
            if eef is not None and math.dist(eef[:2], self.route[-1][:2]) > 0.20:
                t = self.move_tick
                g[2] += (self.z_amp * math.sin(self.z_freq * t + self.z_phase)
                         + self.z_amp2 * math.sin(self.z_freq2 * t + self.z_phase2))
                g[2] = min(max(g[2], WP_Z[0]), WP_Z[1])
            return g
        if self.stage == "ALIGN":
            gx = eef[0] + (self.terminal[0] - p[0])
            gy = eef[1] + (self.terminal[1] - p[1])
            gz = eef[2] + ((self.terminal[2] + 0.035) - p[2])
            return [gx, gy, gz]
        # PLACE — 커넥터 관측 기준 폐루프, z 절반 이득
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
            if self.wait >= self.grip_wait:
                self.wait = 0
                if not gripping:
                    self.no_grip += 1
                    if self.no_grip < GRIP_LOST_STEPS:
                        self.wait = self.grip_wait - 1
                        return [0.0, 0.0, 0.0, *rot], True, info
                    self.escape_goal = [eef[0], eef[1], eef[2] + ESCAPE_H]
                    self.fail_why = "파지 실패"
                    self.stage = "ESCAPE"
                    return [0.0] * 6, False, {**info, "stage": "ESCAPE"}
                self.no_grip = 0
                self.locked_goal = [self.locked_goal[0], self.locked_goal[1],
                                    self.locked_goal[2] + self.close_end_z + self.lift_h]
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
        limit = self.caps.get(self.stage, MAX_STEP)
        if self.stage == "MOVE" and self.pauses \
                and math.dist(eef[:2], self.route[-1][:2]) > 0.20:
            for t0, t1, depth in self.pauses:
                if t0 <= self.move_tick < t1:
                    limit *= depth
                    info["pause"] = round(depth, 2)
                    break
        delta3 = _clamp([ev * GAIN - DAMP * v for ev, v in zip(err_vec, vel)], limit)

        # 단계 전환 — 픽업 경로의 경계 대기(무동작 구간)를 없앤다. VLA 가
        # 긴 정지 상태를 배우면 추론에서 무한 정지가 날 수 있다 (사용자
        # 지적). 정밀이 필요한 DESCEND(파지 높이)·ALIGN(삽입 정렬)만 짧게
        # 정착하고, TRANSIT·APPROACH·LIFT 는 느슨한 반경으로 관통 통과한다.
        if self.stage == "MOVE":
            self.move_tick += 1
            # 중간 웨이포인트는 정착 없이 즉시 다음으로 — 속도가 붙는다
            if self.wp_i < len(self.route) - 1:
                if err < 0.06:
                    self.wp_i += 1
                return [*delta3, *rot], grip, info
            tol, vmax, need = 0.03, 0.02, 1
        elif self.stage == "ALIGN":
            tol, vmax, need = 0.004, SETTLE_VMAX, 2
        elif self.stage == "TRANSIT":
            tol, vmax, need = 0.05, None, 1
        elif self.stage == "APPROACH":
            tol, vmax, need = 0.015, None, 1
        elif self.stage == "DESCEND":
            tol, vmax, need = POS_TOL, 0.02, 1
        else:                                        # LIFT
            tol, vmax, need = COARSE_TOL, None, 1

        if err < tol and (vmax is None or speed_now < vmax):
            self.hold += 1
            if self.hold >= need:
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
                    self._build_route(list(eef))
                    self.stage = "MOVE"
                elif self.stage == "MOVE":
                    self.stage = "ALIGN"
                elif self.stage == "ALIGN":
                    self.stage = "PLACE"
                elif self.stage == "PLACE":
                    # 부착 이벤트는 드라이버가 본다 — 계속 내려간다
                    pass
                info["stage"] = self.stage
        else:
            self.hold = 0

        return [*delta3, *rot], grip, info
