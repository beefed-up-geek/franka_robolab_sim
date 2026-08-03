# SPDX-License-Identifier: Apache-2.0
"""컨베이어에서 캔을 집어 통에 담는 code-as-policy.

## 제어 — 실측에 기반한 두 개의 이득

액션은 절대 목표가 아니라 **한 스텝 분량의 EEF 증분**이고, 명령한 만큼 다 움직이지
않는다. 실측: 위치 6.9%, 회전 ~8% (RoboLab `DroidRelIKActionCfg` 의 scale=0.5 와
한 제어 스텝 안에서 관절 PD 가 목표까지 못 가는 것이 곱해진 값). 그래서 오차에
비례 이득을 곱해 보상한다. 이득이 1/실현률(≈14)을 넘으면 발산한다.

## 자세 서보 — 반드시 필요하다

회전 델타를 한 번도 보내지 않으면 자세가 조금씩 흘러간다(외란이 무보정 랜덤워크로
쌓인다). 실제로 몇 에피소드 만에 그리퍼가 눈에 띄게 기울었고, 기운 손끝은
내려가면서 캔을 옆으로 밀고(벨트 방향이 가장 무른 축이다), 비뚤게 문 캔은 들자마자
빠졌다. 매 스텝 수직 자세로 서보한다. 회전 명령축은 월드 회전축과 1:1 이다(실측).

## LIFT 는 목표를 잠가야 한다

집은 캔의 파지 자세를 따라 올라가면 목표가 손과 **같이 움직여서** 오차가 영원히
줄지 않는다 — 팔이 천장까지 올라가다 시간 초과로 캔을 떨어뜨렸다. 단계에 들어갈 때
목표를 고정한다. CLOSE 도 같은 이유로 잠근다(닿는 순간 캔이 밀리면 쫓아가며 더 민다).

## 어떤 캔을 집는가

순서는 시뮬레이션이 `order` 로 매겨 준다 (0 = 출구에 가장 가까움, 대기열은 -1).
맨 앞만 계속 집으면 시연이 단조로우니 1·2번째를 섞되, 앞선 캔이 출구에 가까우면
반드시 그것부터 — 안 놓치는 선에서 최대한 섞는다.
"""
from __future__ import annotations

import math
import random

# ── 제어 이득 (실측 6.9%/8% 실현률 보상) ──────────────────────────────
GAIN = 2.5              # 위치 비례 이득. 14 이상은 발산, 4는 진동했다
MAX_STEP = 0.10         # 한 스텝 최대 위치 명령 [m]
# 속도 감쇠 이득 [명령 / (m/스텝)]. P 제어에는 브레이크가 없다 — 먼 목표로는
# 명령이 상한에 포화된 채 달려가므로 도착 속도가 그대로 오버슛이 되고, 반대쪽에서
# 다시 포화되어 **한 번 흔들리면 영영 멈추지 않는다.** 실측: APPROACH 에서
# ±13cm·주기 4초 진동이 150초 타임아웃 내내 지속됐다. GAIN 을 2.5 로 낮춰 둔
# 것도 같은 문제의 응급처치였는데, 캔이 2종일 때는 첫 캔이 가까워(y≈-0.03)
# 임계 아래였고, 4종이 되며 첫 캔이 출구 근처(y≈0.21)로 밀려 이동이 길어지자
# 임계를 넘었다 — 이득이 아니라 **감쇠가 없는 것**이 병이다.
# 실측 EEF 속도에 비례해 명령을 깎으면 접근 속도가 오차에 비례하게 되어
# (평형에서 v ≈ GAIN·err/(DAMP + 1/REALIZED)) 목표 앞에서 저절로 감속한다.
# 원거리 순항은 1/REALIZED(≈3.3) 항이 지배해 4 를 더해도 20% 쯤만 느려진다.
DAMP = 4.0
# 도달 판정에 오차와 **함께** 보는 속도 상한 [m/스텝]. 오차가 공 안이어도 팔이
# 아직 움직이는 중이면 지나가는 중일 뿐 도달이 아니다 — 진동하며 스치는 순간에
# SETTLE_STEPS 가 우연히 차면 그 관성을 다음 단계로 끌고 들어갔다.
# 벨트 추종(피드포워드) 속도가 0.003 m/스텝이라 그보다는 커야 한다.
SETTLE_VMAX = 0.012

# 단계별 속도 상한. 명령을 계속 최대로 주면 팔이 속도를 쌓아 한 스텝에 30mm 씩
# 튄다(실측) — 그 가속에 캔이 손가락에서 빠졌다. 캔을 든 뒤에는 천천히 움직인다.
# 접촉을 유지하는 데 필요한 힘은 0.5N 뿐이지만, 급가속은 그 이상의 관성력을 만든다.
# 처리량이 목적이므로 상한을 실측에 맞춰 올렸다. 캔이 13초마다 도착하는데
# 한 번 집는 데 45초가 걸리면 구조적으로 밀린다 — 운반(17.7초)과 이동(7.3초)이
# 절반이었다. 잡은 뒤 놓치지 않을 만큼만 남기고 나머지는 빠르게 간다.
STAGE_MAX_STEP = {
    "DESCEND": 0.06,    # 캔을 치지 않게
    "LIFT": 0.05,       # 0.02 는 과했다 — 감쇠·마찰을 고친 뒤로는 0.05 로도 안 놓친다
    "TO_BIN": 0.10,     # 0.06~0.08 로 낮춰도 놓침이 줄지 않았다. 실측 최적값
    "TRANSIT": 0.18,    # 빈손 수평 이동 — 제일 빠르게
}
ROT_GAIN = 3.0          # 자세 비례 이득
MAX_ROT = 0.25          # 한 스텝 최대 회전 명령 [rad]
# 캔을 든 동안의 회전 상한. 자세 서보는 계속 돌려야 하지만(안 그러면 조금씩 기운다),
# 운반 중 큰 회전은 캔을 흔들어 손가락에서 빼낸다.
MAX_ROT_CARRY = 0.05
POS_TOL = 0.007         # 파지 단계 도달 판정 [m].
                        # 0.012 로 늘렸더니 그리퍼가 캔 중심에서 벗어난 채 닫혀
                        # 파지 실패가 급증했다(1/10). 정밀도를 포기하면 안 된다.
RETREAT_TOL = 0.03      # 물러나기만 하는 단계는 느슨하게
COARSE_TOL = 0.02       # 파지와 무관한 단계 — 이동·운반·투하
ESCAPE_H = 0.12         # 실패 시 수직으로 빠져나오는 높이 [m]
SETTLE_STEPS = 3

# 수직으로 내려 잡는 자세 — 월드 +Y축 90° 회전 (로컬 +X 가 바닥을 본다)
VERTICAL_QUAT = (0.7071068, 0.0, 0.7071068, 0.0)

APPROACH_H = 0.11       # 파지점 위 접근 높이 [m]
# 수평 이동은 **항상 이 높이에서** 한다 [m, 플랜지 기준]. 통 위(0.41)에서 곧바로
# 다음 목표로 대각선 이동하면 손가락이 벨트 위 캔들을 쓸어 떨어뜨린다
# (실측: 접근 중 목표가 벨트 밖으로 밀려나는 실패가 4/6).
# 0.60 은 안전 반경(0.85m)에 걸려 벨트 끝(y=0.3)에서 팔이 닿지 못했다 —
# sqrt(0.52² + 0.3² + 0.60²) = 0.85. 0.50 이면 0.78 로 여유가 있고, 손끝은
# 0.35 라 가장 높은 캔 윗면(0.26)보다 9cm 위다.
TRANSIT_Z = 0.50
# 집은 뒤 **추가로** 들어 올리는 높이 [m] — CLOSE 가 이미 올려 놓은 54mm 위에서
# 이어서 올린다(CLOSE_END_Z 참고). 파지점 기준 절대 높이가 아니다.
#
# 절대 높이로 두었을 때 캔을 번쩍 들었다가 24mm 도로 내려놓았다. CLOSE 가 대기 대신
# 상승하도록 바꾸면서 CLOSE 가 끝나는 높이가 파지점 +54mm 가 되었는데, LIFT 는
# 여전히 파지점에서 +30mm 를 다시 재고 있었기 때문이다.
#
# 결과적으로 손끝이 0.305 까지 올라간다. 가장 키 큰 파열 캔의 윗면이 0.283 이라
# 그 위를 지나가고, 이후 TO_BIN 이 통 높이(0.26)로 내리며 옮긴다.
LIFT_H = 0.03
# 그리퍼를 닫는 동안 머무는 스텝 수 (6Hz 기준 18스텝 ≈ 3초). Robotiq 손가락이
# 다 닫히기 전에 들어 올리면 캔이 빠지고, 상태 토픽은 명령값이라 실제 손가락
# 피드백이 없어 시간으로 기다리는 수밖에 없다. 12 로 줄였더니 파지 성공률이
# 6/9 에서 1/14 로 무너졌다 — 이 시간은 깎을 수 없다.
#
# 대신 그 동안 **가만히 있지 않게** 한다. VLA 는 액션을 청크 단위로 예측하므로
# 정지 구간이 청크(16스텝)보다 길면 학습된 정책이 정지 청크를 계속 뱉으며 무한
# 대기에 빠진다(흡수 상태). 실측에서 CLOSE 정지가 15~20스텝으로 전체 정지의
# 84% 였다 — 아래 CLOSE_PRESS/CLOSE_RISE 가 대기를 움직임으로 바꾼다.
GRIP_WAIT = 18
# 닫는 동안의 움직임. 앞부분은 살짝 눌러 손가락을 앉히고, 뒷부분은 천천히 들어
# 올리기 시작한다. 그러면 18스텝 내내 EEF 가 계속 움직여 **정지 구간이 생기지
# 않는다** — VLA 가 청크째로 정지에 갇히는 것을 막는 것이 목적이다.
#
# 눌림은 6mm 로 제한한다. 32mm 를 줬더니 손끝이 벨트(0.200)를 뚫어 14회 연속
# 파지에 실패했다. 파지점이 벨트 위 21mm 뿐이라 그 이상은 안 된다.
CLOSE_PRESS_MAX = 0.006     # 앞부분 총 눌림 [m]
# 눌림에 쓰는 스텝 수. 6스텝(1mm/스텝)으로 나눠 눌렀더니 실제 이동이 0.3mm 라
# 그 구간이 통째로 정지로 남았다. 2스텝(3mm/스텝)으로 몰아 누르고 바로 상승으로
# 넘어가면 CLOSE 안의 정지가 3~4스텝으로 줄어든다.
CLOSE_PRESS_STEPS = 2
# 이후 한 스텝에 들어 올리는 높이 [m].
# 1.5mm 로는 실제 이동이 0.5mm 라 여전히 "정지" 로 잡혔다 — 팔이 명령의 일부만
# 따라가기 때문이다. 4mm 면 실제로도 1mm 넘게 움직여 정지 구간이 끊긴다.
# 12스텝이면 48mm 라, 사실상 **닫으면서 들어 올리기 시작**하는 셈이다.
CLOSE_RISE = 0.004
# CLOSE 가 끝나는 순간의 목표 높이 — **파지점 기준**. 눌렀다가 쭉 올린 결과다.
#
# 이 값이 필요한 이유: LIFT 는 목표를 파지점에서 다시 계산하는데, CLOSE 가 이미
# 그보다 높이 올려 놓았다는 것을 모르면 목표가 **아래로 떨어진다.** 실제로
# CLOSE 끝 +54mm → LIFT 목표 +30mm 이 되어, 캔을 번쩍 들었다가 24mm 도로 내려
# 놓는 동작이 나왔다. 들어 올리는 도중에 내려놓는 시연은 VLA 에 그대로 학습된다.
CLOSE_END_Z = CLOSE_RISE * (GRIP_WAIT - 1 - CLOSE_PRESS_STEPS) - CLOSE_PRESS_MAX
OPEN_WAIT = 3            # 통 위에서 놓는 것이라 짧아도 된다

# 피드포워드에 쓰는 한 스텝 시간 [s]. 제어율 6Hz 기준.
BELT_FF_DT = 1.0 / 6.0

# 목표를 벨트 진행 방향으로 앞질러 잡는 시간 [s].
# 이론상 지연은 속도/유효이득 ≈ 17mm 로 봤는데 **실측 추종 오차는 3~6mm** 였다.
# 17mm 를 앞지르니 오차가 허용치(7mm) 밑으로 내려가지 못해 DESCEND 가 끝나지
# 않았고, 그 사이 캔이 출구로 빠졌다. 실측에 맞춰 거의 0 으로 둔다.
LEAD_S = 0.2

# 실현률 — 명령한 델타 중 실제로 움직이는 비율(실측 6.9%)과 제어율.
# 벨트 속도만큼 손을 **같이** 움직이려면 명령을 그만큼 크게 줘야 한다:
#   명령[m/스텝] = 속도[m/s] / (실현률 x 제어율)
# 이 항이 없으면 비례 제어만으로 따라가느라 항상 뒤처지고, 무는 순간 패드가
# 캔의 뒤쪽을 밀어 앞으로 튕겨낸다 — 파지 실패의 주 원인이었다.
# 6.9% 는 **단발 명령**의 정상상태 비율이다. 같은 명령을 계속 주면 팔이 속도를
# 쌓아 실제로는 30% 가까이 움직인다(실측: 명령 0.10 에 한 스텝 28~33mm).
# 6.9% 로 잡으면 4배 과보정이라 손이 캔을 앞질러 가 버렸다.
REALIZED = 0.30
RATE_HZ = 6.0

# 파지를 이만큼 더 깊게 내린다 [m]. 손끝을 캔 중심에 정확히 두면 패드가 캔의 위쪽
# 절반만 무는데, 8mm 더 내리면 가장 불룩한 허리를 물어 미끄러짐이 줄어든다.
# 제일 납작한 참치캔(중심이 벨트 위 16mm)에서도 손끝이 벨트에 8mm 남는다.
# 손끝을 캔 중심보다 이만큼 아래에 둔다 [m].
# 12mm 까지 내려 패드를 더 걸쳐 보았지만 파지 실패만 늘었다 — 깊이보다 중심
# 정렬이 중요하다. 8mm 가 실측 최적값이다.
GRASP_DEPTH = 0.008

# 반송면 높이와, 손끝이 벨트에서 최소한 띄워야 하는 간격 [m].
BELT_TOP_Z = 0.200
MIN_TCP_CLEAR = 0.004
# LIFT 를 마칠 때 캔이 반송면보다 이만큼 올라와 있어야 진짜로 문 것이다 [m].
LIFTED_MIN = 0.04

# 납작한 캔(참치 33mm, 양송이 34mm)은 중심이 벨트 위 16mm 뿐이라, 중심에서 8mm
# 더 내려도 손끝이 벨트 위 8mm 에 머문다. 그러면 37mm 짜리 패드가 캔의 **위쪽
# 절반만** 물어 미끄러진다 — 실측에서 납작한 캔만 연속 5회 파지에 실패했다.
# 그래서 납작한 캔은 손끝을 벨트에 닿기 직전까지 내려 패드를 최대한 걸친다.
FLAT_HALF_H = 0.022     # 반높이가 이보다 작으면 납작한 캔으로 본다

# ── 대상 선택 ─────────────────────────────────────────────────────────
OUTLET_Y = 0.36
# 집기에 필요한 최소 여유 시간 [s]. 접근 → 하강 → 닫기(3초) → 들기까지 걸린다.
# 이보다 늦게 남은 캔은 **애초에 목표로 삼지 않는다** — 쫓아가도 도중에 출구로
# 빠져 시도만 날린다(실측 5회 연속). 놓친 캔은 회수되어 대기열로 돌아간다.
# 실측: TRANSIT(위로+수평) → APPROACH(하강) → DESCEND → CLOSE(3초) → LIFT 까지
# 12~15초가 걸린다. 7초로 잡았더니 접근 도중 목표가 출구로 빠지는 실패가 4/6 였다.
# 집기에 필요한 최소 여유 시간 [s].
#
# 22초로 잡았더니 여유 거리가 37cm 라 **출구에 가까운 캔이 전부 후보에서 빠져**,
# 방금 올라온 캔만 집었다. 목적은 정반대다 — 떨어지기 직전의 캔을 먼저 치워야 한다.
#
# 짧게 잡아도 되는 이유는 그리퍼가 작업 구역에 들어오면 **벨트가 멈추기**
# 때문이다(conveyor.py 의 update_hold). 캔이 흘러가는 것은 로봇이 다가가는
# 동안뿐이라 8초면 충분하다.
NEED_S = 8.0
# 기본은 **맨 앞 캔**이다. 앞의 것을 먼저 치워야 출구로 빠져나가는 것을 막는다.
# 두 번째를 고르는 것은 시연에 변화를 주기 위한 소수 사례로만 남긴다.
SECOND_PROB = 0.10

# 선두 캔이 이만큼 안에 출구에 닿으면 **무조건 그것부터** 집는다 [s].
# 섞어 집는 것은 여유가 있을 때 이야기고, 놓치면 그 캔은 출구를 통과한다.
CRITICAL_S = 30.0

# 목표가 벨트 목록에서 이만큼 연속으로 빠져 있어야 포기한다 [스텝].
LOST_STEPS = 5

# 접촉이 이만큼 연속으로 끊겨야 "놓쳤다" 고 본다 [스텝].
# 접촉력은 솔버 특성상 매 스텝 깜빡인다(실측: 1.2N ↔ 0N 교대). 한 스텝만 보고
# 판단하면 잘 물고 있는데도 ESCAPE 로 빠지면서 **그리퍼를 열어 캔을 떨어뜨린다.**
GRIP_LOST_STEPS = 5
REACH_Y = (-0.30, 0.34)

# ── 통 ────────────────────────────────────────────────────────────────
BIN_XY = (0.26, 0.58)
BIN_DROP_TCP_Z = 0.26


# ── 쿼터니언 (w,x,y,z) ────────────────────────────────────────────────
def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2)


def rot_error(current, target=VERTICAL_QUAT):
    """current → target 회전을 월드 회전 벡터로."""
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


class TargetPicker:
    def __init__(self, seed: int = 0) -> None:
        self.rng = random.Random(seed)
        self.last_choice = None

    def choose(self, cans: list[dict], belt_order: list[str],
               belt_mps: float = 0.0) -> dict | None:
        """belt_order 는 시뮬레이션이 준 **벨트 위 화물의 진행 순서**다.

        여기 없는 화물은 대기열(상판 아래)이거나 굴러떨어진 것이라 후보가 아니다.
        전에는 objects 배열에서 골랐더니 상판 아래로 초기화된 캔을 집으러 팔이
        내려가는 일이 있었다.
        """
        by_name = {c["name"]: c for c in cans}
        margin = belt_mps * NEED_S           # 멈춘 벨트면 0 — 시간 제약이 없다
        usable = []
        for name in belt_order:                       # 이미 출구에 가까운 순서
            c = by_name.get(name)
            if c is None:
                continue
            y = c["pos"][1]
            if not (REACH_Y[0] <= y <= REACH_Y[1]):
                continue
            if OUTLET_Y - y < margin:                 # 잡기 전에 빠져나간다
                continue
            usable.append(c)
        if not usable:
            self.last_choice = None
            return None
        if len(usable) < 2:
            self.last_choice = 0
            return usable[0]
        # 선두가 위험하면 섞지 않는다.
        if belt_mps > 0 and (OUTLET_Y - usable[0]["pos"][1]) / belt_mps < CRITICAL_S:
            self.last_choice = 0
            return usable[0]
        idx = 1 if self.rng.random() < SECOND_PROB else 0
        self.last_choice = idx
        return usable[idx]


class PickPlacePolicy:
    """집어서 통에 담는 상태 기계."""

    def __init__(self, seed: int = 0) -> None:
        self.picker = TargetPicker(seed)
        # 속도 추정용 직전 EEF. reset() 에서 지우지 않는다 — 시도 사이에도 팔은
        # 이어서 움직이고 있으므로, 지우면 그 한 스텝만 감쇠가 빠진다.
        self.prev_eef = None
        self.reset()

    def _begin_escape(self, eef, why: str) -> None:
        """실패 사유를 안고 ESCAPE 로 넘어간다."""
        self.stage = "ESCAPE"
        self.escape_goal = [eef[0], eef[1], eef[2] + ESCAPE_H]
        self.fail_why = why
        self.hold = 0
        self.wait = 0

    def reset(self) -> None:
        self.stage = "SEARCH"
        self.target_name = None
        self.hold = 0
        self.wait = 0
        self.picked_index = None
        self.locked_goal = None      # CLOSE/LIFT 에서 잠근 목표
        self.lead = 0.0              # 벨트 진행 방향 예측 보정 [m]
        self.escape_goal = None
        self.fail_why = None
        self.ff_y = 0.0
        self.lost = 0
        self.no_grip = 0
        self.grasp_z = None          # 파지 시점의 캔 높이 (낙하 감지)

    def _escape(self, eef):
        """실패했을 때 **수직으로만** 빠져나오는 목표.

        옆으로 움직이며 손을 빼면 벌어진 손가락이 캔을 밀어 벨트 밖으로 떨어뜨린다
        (실측: 한 세션에 5개가 떨어졌다). 곧바로 SEARCH 로 가면 다음 목표를 향해
        수평 이동을 시작하므로, 그 전에 위로 뽑아 낸다.
        """
        return [eef[0], eef[1], eef[2] + ESCAPE_H]

    def _grasp_flange_z(self, can, flange_offset):
        """파지 시 플랜지 높이. 손끝(TCP)이 벨트를 뚫지 않는 선에서 최대한 내린다."""
        center_z = can["flange"][2] - flange_offset
        if can.get("half_height", 0.03) < FLAT_HALF_H:
            tcp_z = BELT_TOP_Z + MIN_TCP_CLEAR          # 납작한 캔 — 바닥까지
        else:
            tcp_z = center_z - GRASP_DEPTH
        tcp_z = max(tcp_z, BELT_TOP_Z + MIN_TCP_CLEAR)
        return tcp_z + flange_offset

    def _goal(self, can, eef, flange_offset):
        """단계별 목표 플랜지 위치. 잠글 단계는 잠근 값을 쓴다."""
        if self.stage == "TRANSIT":
            f = can["flange"]
            return [f[0], f[1] + self.lead, TRANSIT_Z]
        if self.stage == "APPROACH":
            f = can["flange"]
            return [f[0], f[1] + self.lead, f[2] + APPROACH_H]
        if self.stage == "DESCEND":
            f = can["flange"]
            return [f[0], f[1] + self.lead, self._grasp_flange_z(can, flange_offset)]
        if self.stage in ("CLOSE", "LIFT"):
            return self.locked_goal
        if self.stage in ("TO_BIN", "OPEN"):
            return [BIN_XY[0], BIN_XY[1], BIN_DROP_TCP_Z + flange_offset]
        return [BIN_XY[0], BIN_XY[1], BIN_DROP_TCP_Z + flange_offset + LIFT_H]  # RETREAT

    def act(self, eef, eef_quat, cans, flange_offset, belt_mps=0.0, gripping=False,
            belt_order=None):
        """(delta6, gripper_close, info) — delta 는 월드 기준 한 스텝 증분.

        belt_mps  벨트 속도 [m/s]. 손가락이 닫히는 동안에도 캔은 흘러가므로 잠근
                  목표를 같은 속도로 밀어 준다. 안 그러면 3초 대기 중 캔이 5cm
                  지나가 손가락 사이에서 빠진다.
        gripping  캔에 접촉력이 걸려 있는가. 물린 뒤에는 벨트가 그 캔을 놓아
                  주므로(conveyor.py) 피드포워드를 멈춰야 한다.
        """
        info = {"stage": self.stage, "target": self.target_name}
        # 실측 EEF 속도 [m/스텝] — 감쇠 항과 도달 판정에 쓴다. 어떤 분기로 빠지든
        # 매 스텝 갱신되도록 맨 앞에서 계산한다. 한 번이라도 건너뛰면 다음 속도가
        # 두 스텝 치로 계산되어 그 스텝만 브레이크가 두 배로 걸린다.
        vel = ([e - p for e, p in zip(eef, self.prev_eef)]
               if self.prev_eef is not None else [0.0, 0.0, 0.0])
        self.prev_eef = list(eef)
        speed = math.sqrt(sum(v * v for v in vel))
        carrying = self.stage in ("CLOSE", "LIFT", "TO_BIN", "OPEN")
        rot = _clamp([v * ROT_GAIN for v in rot_error(eef_quat)],
                     MAX_ROT_CARRY if carrying else MAX_ROT) \
            if eef_quat else [0.0, 0.0, 0.0]

        belt_order = belt_order or []
        self.lead = belt_mps * LEAD_S
        # 벨트를 따라가기 위한 속도 피드포워드 [m/스텝 명령]
        self.ff_y = belt_mps / (REALIZED * RATE_HZ)
        if self.stage == "SEARCH":
            pick = self.picker.choose(cans, belt_order, belt_mps)
            if pick is None:
                return [0.0, 0.0, 0.0, *rot], False, info
            self.target_name = pick["name"]
            self.picked_index = self.picker.last_choice
            self.stage = "TRANSIT"
            info.update(stage=self.stage, target=self.target_name,
                        choice=self.picked_index)
            return [0.0, 0.0, 0.0, *rot], False, info

        if self.stage == "ESCAPE":
            d3 = _clamp([(g - e) * GAIN - DAMP * v
                         for g, e, v in zip(self.escape_goal, eef, vel)], MAX_STEP)
            err = math.sqrt(sum((g - e) ** 2 for g, e in zip(self.escape_goal, eef)))
            if err < RETREAT_TOL:
                why = self.fail_why
                self.reset()
                return [0.0] * 6, False, {**info, "stage": "SEARCH",
                                          "abort": True, "why": why}
            return [*d3, *rot], False, {**info, "err": round(err, 4)}

        can = next((c for c in cans if c["name"] == self.target_name), None)
        # 잡기 전 단계에서는 목표가 **벨트 순서 목록에 남아 있어야** 한다. 빠졌다는
        # 것은 회수됐거나 굴러떨어졌다는 뜻이고, 그 자리를 쫓아가면 상판 아래로
        # 팔을 뻗게 된다. DESCEND 는 손끝이 닿으며 판정이 흔들릴 수 있어 높이로도 본다.
        if can is None:
            self.reset()
            return [0.0] * 6, False, {**info, "stage": "SEARCH", "abort": True,
                                      "why": "목표가 목록에서 사라짐"}
        # 한 번 정한 목표는 **끝까지 밀고 간다.** 매 스텝 후보를 다시 고르면
        # 벨트가 흐르며 순위가 바뀔 때마다 목표가 갈아치워져, 뒤쪽 캔으로 내려가다
        # 갑자기 앞쪽 캔으로 방향을 트는 일이 생긴다. 목표가 아예 회수됐을 때만
        # 포기한다.
        if self.stage in ("TRANSIT", "APPROACH"):
            if self.target_name in belt_order:
                self.lost = 0
            else:
                # 벨트 위 판정은 캔이 살짝 튀기만 해도 한두 스텝 흔들린다. 그때마다
                # 목표를 버리면 뒤쪽 캔으로 가다 앞쪽 캔으로 트는 것처럼 보인다.
                self.lost += 1
                if self.lost >= LOST_STEPS:
                    self.reset()
                    return [0.0] * 6, False, {**info, "stage": "SEARCH",
                                              "abort": True, "why": "목표가 회수됨"}
        if self.stage == "DESCEND" and can["pos"][2] < 0.15:
            self._begin_escape(eef, "목표가 떨어짐")
            return [0.0] * 6, False, {**info, "stage": "ESCAPE", "why": "목표가 떨어짐"}

        grip = self.stage in ("CLOSE", "LIFT", "TO_BIN")
        if self.stage in ("LIFT", "TO_BIN"):
            self.no_grip = 0 if gripping else self.no_grip + 1
            if self.no_grip >= GRIP_LOST_STEPS:
                self._begin_escape(eef, "운반 중 놓침")
                return [0.0] * 6, False, {**info, "stage": "ESCAPE",
                                          "why": "운반 중 놓침"}

        if self.stage in ("CLOSE", "OPEN"):
            if self.stage == "CLOSE" and not gripping and self.locked_goal:
                self.locked_goal[1] += belt_mps * BELT_FF_DT
            self.wait += 1
            if self.wait >= (GRIP_WAIT if self.stage == "CLOSE" else OPEN_WAIT):
                self.wait = 0
                if self.stage == "CLOSE":
                    # 실제로 물었는지 확인하고 넘어간다. 빈손으로 LIFT~OPEN 을 다
                    # 돌면 30초를 버리고, 실패 시연이 기록될 뻔한다.
                    if not gripping:
                        # 대기 끝에 접촉이 안 잡혀도 깜빡임일 수 있다. 몇 스텝
                        # 더 붙잡고 보다가, 그래도 없으면 실패로 처리한다.
                        self.no_grip += 1
                        if self.no_grip < GRIP_LOST_STEPS:
                            self.wait = GRIP_WAIT - 1
                            return [0.0, 0.0, 0.0, *rot], True, info
                        self._begin_escape(eef, "파지 실패")
                        return [0.0] * 6, False, {**info, "stage": "ESCAPE",
                                                  "why": "파지 실패"}
                    self.no_grip = 0
                    # 잠근 파지점 기준 위로. 캔을 따라가면 목표가 손과 같이 움직인다.
                    # CLOSE 가 이미 올려 놓은 높이(CLOSE_END_Z)에서 **이어서** 올린다 —
                    # 파지점에서 다시 재면 목표가 지금 손보다 아래가 되어, 들어 올리던
                    # 캔을 도로 내려놓는다.
                    self.locked_goal = [self.locked_goal[0], self.locked_goal[1],
                                       self.locked_goal[2] + CLOSE_END_Z + LIFT_H]
                    self.stage = "LIFT"
                else:
                    # RETREAT 를 따로 두지 않는다. 다음 TRANSIT 이 어차피 안전
                    # 높이로 올리므로 2.3초를 그냥 버리는 단계였다.
                    self.reset()
                    return [0.0] * 6, False, {**info, "done": True}
            # 제자리에 서 있지 않고 잠근 목표를 따라간다 — 벨트가 흐르면 목표도
            # 같이 흘러서, 그리퍼가 캔과 함께 움직이며 문다.
            gg = list(self.locked_goal or eef)
            if self.stage == "CLOSE":
                # 눌렀다가 들어 올리기 — 대기 내내 EEF 가 움직인다.
                if self.wait <= CLOSE_PRESS_STEPS:
                    gg[2] -= CLOSE_PRESS_MAX * self.wait / CLOSE_PRESS_STEPS
                else:
                    gg[2] -= CLOSE_PRESS_MAX
                    gg[2] += CLOSE_RISE * (self.wait - CLOSE_PRESS_STEPS)
            d3 = _clamp([(g - e) * GAIN for g, e in zip(gg, eef)],
                        STAGE_MAX_STEP.get("TO_BIN", MAX_STEP))
            if self.stage == "CLOSE" and not gripping:
                d3[1] += self.ff_y          # 물기 전까지는 캔과 같이 흘러간다
            return [*d3, *rot], self.stage in ("CLOSE", "LIFT", "TO_BIN"), info

        goal = self._goal(can, eef, flange_offset)
        err_vec = [g - e for g, e in zip(goal, eef)]
        err = math.sqrt(sum(v * v for v in err_vec))
        info["err"] = round(err, 4)
        limit = STAGE_MAX_STEP.get(self.stage, MAX_STEP)
        delta3 = _clamp([ev * GAIN - DAMP * v for ev, v in zip(err_vec, vel)], limit)
        if self.stage in ("TRANSIT", "APPROACH", "DESCEND"):
            delta3[1] += self.ff_y

        # 정밀도가 필요한 것은 캔 바로 위(APPROACH)와 파지점(DESCEND) 뿐이다.
        # TO_BIN 까지 7mm 로 재고 있었더니, 0.4kg 캔을 든 팔이 통 위에서 그 안으로
        # 못 들어와 오차가 ±1cm 로 흔들렸다 — 화면에서는 "박스 위에서 캔을 든 채
        # 뜸을 들이는" 것으로 보인다. 통은 24x38cm 라 2cm 면 충분하다.
        tol = POS_TOL if self.stage in ("APPROACH", "DESCEND") else COARSE_TOL
        # 오차와 속도를 **함께** 본다. 속도를 안 보면 진동하며 공을 스쳐 지나가는
        # 순간에도 도달로 세어져, 그 관성이 다음 단계의 진동이 된다.
        if err < tol and speed < SETTLE_VMAX:
            self.hold += 1
            if self.hold >= SETTLE_STEPS:
                self.hold = 0
                if self.stage == "TRANSIT":
                    self.stage = "APPROACH"
                elif self.stage == "APPROACH":
                    self.stage = "DESCEND"
                elif self.stage == "DESCEND":
                    f = can["flange"]
                    self.locked_goal = [f[0], f[1] + self.lead,
                                        self._grasp_flange_z(can, flange_offset)]
                    self.grasp_z = can["pos"][2]
                    self.stage = "CLOSE"
                elif self.stage == "LIFT":
                    # 접촉 신호는 깜빡여서 "물었다"를 오판한다. 캔이 실제로 벨트에서
                    # 떠올랐는지 높이로 확인하는 것이 진짜 증거다 — 이걸 안 보면
                    # 빈손으로 통까지 갔다가 30초를 버린다.
                    if can["pos"][2] < BELT_TOP_Z + LIFTED_MIN:
                        self._begin_escape(eef, "들리지 않음")
                        return [0.0] * 6, False, {**info, "stage": "ESCAPE",
                                                  "why": "들리지 않음"}
                    self.stage = "TO_BIN"
                elif self.stage == "TO_BIN":
                    self.stage = "OPEN"
                info["stage"] = self.stage
        else:
            self.hold = 0

        return [*delta3, *rot], grip, info
