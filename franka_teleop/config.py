# SPDX-License-Identifier: Apache-2.0
"""텔레오퍼레이션 설정 — 키맵 / 스케일 / 안전 파라미터 / 서버 포트.

액션 규약 (RoboLab DroidRelIKActionCfg): action = [dx, dy, dz, droll, dpitch, dyaw, gripper]
  - 0:3 = EEF 위치 델타 [m]
  - 3:6 = EEF 회전 델타 [rad]
  - 6   = 그리퍼 이진값 (0~1, 0.5 초과면 닫힘)

주의: DroidRelIKActionCfg.arm_action 에 scale=0.5 가 걸려 있어 여기서 정한 델타는
      시뮬레이터 내부에서 절반으로 줄어든다. 아래 값은 그 점을 감안한 최종 체감치다.
"""

# ── 서버 ────────────────────────────────────────────────────────────────
# 외부 115.145.179.126:8003 → 내부 8003 직결. 단일 포트에서 HTML/WS/MJPEG 모두 처리.
HOST = "0.0.0.0"
PORT = 8003

# ── 제어 스케일 ─────────────────────────────────────────────────────────
# 한 스텝(=decimation 적용 후 1 env.step)당 이동량. 너무 크면 IK가 튀고,
# 너무 작으면 조작이 굼뜨다. run_rel_ik_demo.py 의 기본 delta=0.02 를 기준으로 잡음.
# 실측: 상대 IK 는 명령한 델타를 그대로 따라가지 못하고 (DLS 감쇠 + PD 추종 지연)
# 대략 1/3 수준만 반영된다. POS_DELTA=0.012 일 때 실제 속도가 ~1cm/s 로 너무
# 느렸기에 그 점을 감안해 잡은 값이다.
POS_DELTA = 0.050          # [m/step]
ROT_DELTA = 0.070          # [rad/step]

# 배속 — 브라우저에서 [ / ] 로 바꾼다. 큰 이동은 빠르게, 정밀 조작은 느리게.
SPEED_LEVELS = [0.25, 0.5, 1.0, 2.0]
SPEED_DEFAULT_INDEX = 2

# 키를 누르고 있는 동안 델타가 0 → 최대까지 올라가는 데 걸리는 스텝 수.
# 급출발로 인한 IK 점프를 막는다. 1 이면 램프 없음.
RAMP_STEPS = 6

# ── 그리퍼 ──────────────────────────────────────────────────────────────
# robolab.robots.droid.BinaryJointPositionZeroToOneAction 규약:
#   binary_mask = actions > 0.5  →  True 면 close_command
# 즉 0~1 범위이고 0.5 초과가 "닫힘"이다. (-1/+1 이 아니다)
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.0

# ── 안전 파라미터 ───────────────────────────────────────────────────────
# 실물 FR3 텔레오퍼레이션(~/gty_franka/teleoperation/teleop_franka.py)에서 이식.
# 로봇 베이스(panda_link0) 기준 EEF 목표 안전박스 [m].
SAFE_BOX_LO = (0.15, -0.45, 0.02)
SAFE_BOX_HI = (0.75, 0.45, 0.80)
SAFE_RADIUS = 0.85         # 베이스로부터의 최대 반경 [m] — 특이점/도달불가 회피

# ── 영상 스트림 ─────────────────────────────────────────────────────────
STREAM_JPEG_QUALITY = 75
STREAM_MAX_FPS = 20        # 외부망 지연을 고려해 상한을 둔다

# ── 입력 안전장치 ───────────────────────────────────────────────────────
# 브라우저가 죽거나 keyup 패킷이 유실되면 키가 눌린 채로 남아 로봇이 폭주한다.
# 마지막 입력 갱신 이후 이 시간이 지나면 모든 키를 강제 해제한다.
INPUT_TIMEOUT_S = 0.5

# ── 키맵 ────────────────────────────────────────────────────────────────
# Isaac Lab Se3Keyboard 관례를 따른다. 값 = (액션 인덱스, 부호)
KEY_MAP = {
    "KeyW": (0, +1.0),   # +X (앞)
    "KeyS": (0, -1.0),   # -X (뒤)
    "KeyA": (1, +1.0),   # +Y (좌)
    "KeyD": (1, -1.0),   # -Y (우)
    "KeyQ": (2, +1.0),   # +Z (위)
    "KeyE": (2, -1.0),   # -Z (아래)
    "KeyZ": (3, +1.0),   # roll +
    "KeyX": (3, -1.0),   # roll -
    "KeyT": (4, +1.0),   # pitch +
    "KeyG": (4, -1.0),   # pitch -
    "KeyC": (5, +1.0),   # yaw +
    "KeyV": (5, -1.0),   # yaw -
}

# 눌림 상태가 아니라 1회성 이벤트로 처리하는 키
KEY_GRIPPER_TOGGLE = "Space"
KEY_RESET = "KeyR"
KEY_SPEED_DOWN = "BracketLeft"
KEY_SPEED_UP = "BracketRight"
