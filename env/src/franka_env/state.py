# SPDX-License-Identifier: Apache-2.0
"""시뮬레이션 스레드와 웹 서버 스레드가 공유하는 상태.

Isaac Sim(Omniverse Kit)은 반드시 메인 스레드에서 돌아야 하므로 aiohttp 서버를
별도 스레드에 띄운다. 두 스레드가 만나는 지점은 이 클래스 하나뿐이며, 모든
접근은 단일 Lock으로 직렬화한다.

  웹 스레드 → on_key()/orbit()/zoom()   (입력)
  심 스레드 → consume()/publish_frame() (제어 + 영상)

키 눌림 상태와 하트비트는 **클라이언트별로** 따로 관리한다. 전역으로 두면 다른
탭이 보내는 하트비트가 조작 중인 클라이언트의 입력 타임아웃을 갱신해버려서,
조작하던 브라우저가 죽어도 로봇이 계속 움직인다.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

from franka_env import config


@dataclass
class _Client:
    """한 브라우저 연결의 입력 상태."""
    pressed: set[str] = field(default_factory=set)
    held_steps: dict[str, int] = field(default_factory=dict)
    last_input: float = field(default_factory=time.monotonic)

    def stale(self) -> bool:
        return time.monotonic() - self.last_input > config.INPUT_TIMEOUT_S


class TeleopState:
    def __init__(self, belt_mpm: float | None = None, view: str = "behind") -> None:
        """
        Args:
            view: 시점 프리셋 이름 (config.VIEW_PRESETS). F 키도 여기로 되돌린다.
            belt_mpm: 컨베이어 초기 속도 [m/분]. 기본 단계에 없는 값이면 목록에
                끼워 넣는다 — 실험 조건을 임의의 값으로 잡을 수 있어야 하고,
                그러면서도 키(, .)로 단계를 오갈 수 있어야 하기 때문이다.
        """
        self._lock = threading.Lock()

        self._clients: dict[int, _Client] = {}

        # 1회성 이벤트 / 전역 설정 — 여러 클라이언트가 공유해도 무방하다.
        self._gripper = config.GRIPPER_OPEN
        # 대기 중인 리셋 요청의 **강도** (config.RESET_LEVELS) 또는 None.
        # 불리언이 아닌 이유: 브라우저 키·ROS·HTTP 가 서로 다른 강도로 요청할 수
        # 있어야 프로세스를 죽이지 않고도 어떤 상태에서든 빠져나올 수 있다.
        self._reset_level: str | None = None
        # ROS 등 외부에서 들어온 명령. 키보드 델타와 **더해진다** — 사람이 잡고
        # 있는 중에도 정책이 밀어붙일 수 있어야 원격 개입 실험이 된다.
        # 한 번 쓰면 지운다. 안 지우면 퍼블리셔가 멈춰도 로봇이 계속 흐른다.
        self._ext_delta: list[float] | None = None
        self._speed_idx = config.SPEED_DEFAULT_INDEX

        # 카메라 궤도 — CAM_TARGET 을 중심으로 한 구면좌표
        self._view = config.VIEW_PRESETS.get(view, config.VIEW_PRESETS["behind"])
        self._cam_az, self._cam_el, self._cam_radius = self._view

        # 컨베이어 — 심 스레드가 consume_belt() 로 변경분을 가져간다.
        levels = list(config.BELT_SPEED_MPM)
        idx = config.BELT_SPEED_DEFAULT_INDEX
        if belt_mpm is not None:
            if belt_mpm not in levels:
                levels.append(belt_mpm)
                levels.sort()
            idx = levels.index(belt_mpm)
        self._belt_mpm = levels
        self._belt_ms = [v / 60.0 for v in levels]
        self._belt_idx = idx
        self._belt_on = True
        self._belt_dirty = True

        # 영상 — 화면 이름별로 따로 들고 있는다 (view / front / wrist).
        self._frames: dict[str, tuple[bytes, int]] = {}
        self._frame_seq = 0

        # UI 표시용 텔레메트리
        self._telemetry: dict = {}

    # ── 웹 스레드 ────────────────────────────────────────────────────────
    def add_client(self, client_id: int) -> None:
        with self._lock:
            self._clients[client_id] = _Client()

    def remove_client(self, client_id: int) -> None:
        with self._lock:
            self._clients.pop(client_id, None)

    @property
    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)

    def on_key(self, client_id: int, code: str, down: bool) -> None:
        with self._lock:
            client = self._clients.get(client_id)
            if client is None:
                return
            client.last_input = time.monotonic()

            if code == config.KEY_GRIPPER_TOGGLE:
                # 토글은 눌림 시작에만 반응 — 누르고 있어도 한 번만 뒤집힌다.
                if down and code not in client.pressed:
                    self._gripper = (
                        config.GRIPPER_CLOSE
                        if self._gripper == config.GRIPPER_OPEN
                        else config.GRIPPER_OPEN
                    )
            elif code == config.KEY_RESET:
                if down and code not in client.pressed:
                    self._escalate_reset(config.RESET_DEFAULT)
            elif code == config.KEY_RESET_FULL:
                if down and code not in client.pressed:
                    self._escalate_reset("full")
            elif code == config.KEY_VIEW_RESET:
                if down and code not in client.pressed:
                    self._cam_az, self._cam_el, self._cam_radius = self._view
            elif code == config.KEY_BELT_TOGGLE:
                if down and code not in client.pressed:
                    self._belt_on = not self._belt_on
                    self._belt_dirty = True
            elif code in (config.KEY_BELT_SLOWER, config.KEY_BELT_FASTER):
                if down and code not in client.pressed:
                    step = -1 if code == config.KEY_BELT_SLOWER else +1
                    self._belt_idx = max(
                        0, min(len(self._belt_ms) - 1, self._belt_idx + step)
                    )
                    self._belt_dirty = True
            elif code in (config.KEY_SPEED_DOWN, config.KEY_SPEED_UP):
                if down and code not in client.pressed:
                    step = -1 if code == config.KEY_SPEED_DOWN else +1
                    self._speed_idx = max(
                        0, min(len(config.SPEED_LEVELS) - 1, self._speed_idx + step)
                    )

            if down:
                client.pressed.add(code)
            else:
                client.pressed.discard(code)
                client.held_steps.pop(code, None)

    # ── 리셋 요청 ────────────────────────────────────────────────────────
    def _escalate_reset(self, level: str) -> str:
        """대기 중인 리셋 요청의 강도를 올린다. **락을 쥔 채로** 부른다.

        강한 쪽이 이긴다. 브라우저가 R(soft) 을 누른 직후 수집기가 full 을
        요청했다면 full 로 나가야 한다 — 약한 요청이 먼저 소비되어 버리면
        정작 필요한 초기화가 한 스텝 늦어지고, 그 사이 심이 또 터진다.
        """
        if level not in config.RESET_LEVELS:
            level = config.RESET_DEFAULT
        cur = self._reset_level
        if cur is None or config.RESET_LEVELS.index(level) > config.RESET_LEVELS.index(cur):
            self._reset_level = level
        return self._reset_level

    def request_reset(self, level: str = config.RESET_DEFAULT) -> str:
        """외부(ROS·HTTP)에서 리셋을 요청한다. 실제로 잡힌 강도를 돌려준다.

        심 스레드가 다음 스텝에 consume() 으로 가져가 처리한다. 여기서 직접
        리셋하지 않는 이유는 물리 상태를 만지는 일이 전부 심 스레드 소유이기
        때문이다 — 웹/ROS 스레드에서 write_*_to_sim 을 부르면 조용히 깨진다.
        """
        with self._lock:
            return self._escalate_reset(level)

    def consume_external(self) -> list[float] | None:
        """외부(ROS) 델타를 꺼내 간다. 한 번 쓰면 지운다.

        키보드 델타와 **따로** 내보내는 이유는 좌표계가 다르기 때문이다. 키보드는
        화면 기준이라 runner 가 카메라 방위각으로 돌려서 쓰는데, ROS 명령은 월드
        기준이므로 그 회전을 타면 안 된다. 섞어서 반환했더니 팔이 명령과 반대
        방향으로 갔다.
        """
        with self._lock:
            d, self._ext_delta = self._ext_delta, None
            return d

    def set_external_delta(self, delta: list[float]) -> None:
        """외부에서 한 스텝 분량의 EEF 델타를 넣는다 (ROS /cmd/eef_delta)."""
        with self._lock:
            self._ext_delta = [float(v) for v in delta[:6]]

    def set_external_gripper(self, close: bool) -> None:
        with self._lock:
            self._gripper = config.GRIPPER_CLOSE if close else config.GRIPPER_OPEN

    def release_all(self, client_id: int) -> None:
        """해당 클라이언트의 키만 뗀다 (창 포커스 이탈·연결 종료)."""
        with self._lock:
            client = self._clients.get(client_id)
            if client is not None:
                client.pressed.clear()
                client.held_steps.clear()

    def heartbeat(self, client_id: int) -> None:
        with self._lock:
            client = self._clients.get(client_id)
            if client is not None:
                client.last_input = time.monotonic()

    def orbit(self, dx: float, dy: float) -> None:
        """마우스 드래그 → 방위각/고도각. 화면을 끄는 방향으로 시점이 따라오게 부호를 잡는다."""
        with self._lock:
            self._cam_az -= dx * config.CAM_ORBIT_SENS
            # 수직에 가까워지면 look-at 의 up 벡터가 특이해져 화면이 뒤집힌다.
            self._cam_el = max(
                config.CAM_ELEV_MIN,
                min(config.CAM_ELEV_MAX, self._cam_el + dy * config.CAM_ORBIT_SENS),
            )

    def zoom(self, delta: float) -> None:
        """휠 → 거리. 비율로 곱해야 멀 때는 크게, 가까울 때는 곱게 움직인다."""
        with self._lock:
            factor = math.exp(delta * config.CAM_ZOOM_SENS)
            self._cam_radius = max(
                config.CAM_RADIUS_MIN, min(config.CAM_RADIUS_MAX, self._cam_radius * factor)
            )

    def camera_eye(self) -> tuple[float, float, float]:
        """현재 구면좌표를 월드 카메라 위치로 변환한다."""
        with self._lock:
            az, el, r = self._cam_az, self._cam_el, self._cam_radius
        tx, ty, tz = config.CAM_TARGET
        return (
            tx + r * math.cos(el) * math.cos(az),
            ty + r * math.cos(el) * math.sin(az),
            tz + r * math.sin(el),
        )

    def set_belt_mpm(self, mpm: float) -> None:
        """벨트 속도를 직접 지정한다 [m/분] — ROS /cmd/belt 가 부른다.

        키보드 단계 목록에 없는 값이면 목록에 끼워 넣는다. 수집기가 에피소드마다
        무작위 속도를 주는 용도라, 단계가 늘어나는 것은 문제되지 않는다.
        """
        mpm = max(0.0, float(mpm))
        with self._lock:
            if mpm not in self._belt_mpm:
                self._belt_mpm.append(mpm)
                self._belt_mpm.sort()
                self._belt_ms = [v / 60.0 for v in self._belt_mpm]
            self._belt_idx = self._belt_mpm.index(mpm)
            self._belt_dirty = True

    def consume_belt(self) -> tuple[float, bool] | None:
        """벨트 설정이 바뀌었을 때만 (속도, 켜짐) 을 돌려준다. 아니면 None."""
        with self._lock:
            if not self._belt_dirty:
                return None
            self._belt_dirty = False
            return self._belt_ms[self._belt_idx], self._belt_on

    def camera_azimuth(self) -> float:
        with self._lock:
            return self._cam_az

    def get_frame(self, name: str = "view") -> tuple[bytes | None, int]:
        with self._lock:
            frame, frame_id = self._frames.get(name, (None, -1))
            return frame, frame_id

    def get_telemetry(self) -> dict:
        with self._lock:
            return dict(self._telemetry)

    # ── 심 스레드 ────────────────────────────────────────────────────────
    def consume(self) -> tuple[list[float], float, str | None]:
        """이번 스텝의 (6축 델타, 그리퍼, 리셋강도)를 계산해 돌려준다.

        리셋강도는 요청이 없으면 None, 있으면 config.RESET_LEVELS 중 하나다.
        """
        with self._lock:
            delta = [0.0] * 6
            speed = config.SPEED_LEVELS[self._speed_idx]

            for client in self._clients.values():
                # 입력이 끊긴 클라이언트는 폭주 방지를 위해 키를 전부 놓은 것으로 본다.
                if client.stale():
                    client.pressed.clear()
                    client.held_steps.clear()
                    continue

                for code in client.pressed:
                    mapping = config.KEY_MAP.get(code)
                    if mapping is None:
                        continue
                    idx, sign = mapping

                    # 누르고 있는 동안 0 → 1 로 서서히 올려 급출발을 막는다.
                    held = client.held_steps.get(code, 0) + 1
                    client.held_steps[code] = held
                    ramp = min(1.0, held / max(1, config.RAMP_STEPS))

                    scale = config.POS_DELTA if idx < 3 else config.ROT_DELTA
                    delta[idx] += sign * scale * ramp * speed

            # 두 클라이언트가 같은 방향을 눌러도 속도가 배가 되지 않게 자른다.
            limit_pos = config.POS_DELTA * speed
            limit_rot = config.ROT_DELTA * speed
            for i in range(6):
                limit = limit_pos if i < 3 else limit_rot
                delta[i] = max(-limit, min(limit, delta[i]))

            reset, self._reset_level = self._reset_level, None
            return delta, self._gripper, reset

    def publish_frame(self, jpeg: bytes, name: str = "view") -> None:
        with self._lock:
            self._frame_seq += 1
            self._frames[name] = (jpeg, self._frame_seq)

    def publish_telemetry(self, **kwargs) -> None:
        with self._lock:
            self._telemetry.update(kwargs)
            self._telemetry["gripper"] = (
                "CLOSED" if self._gripper == config.GRIPPER_CLOSE else "OPEN"
            )
            self._telemetry["speed"] = config.SPEED_LEVELS[self._speed_idx]
            self._telemetry["cam_radius"] = round(self._cam_radius, 2)
            speed = self._belt_ms[self._belt_idx]
            mpm = self._belt_mpm[self._belt_idx]
            # 내부 계산은 m/s 지만 표시는 실물 관례대로 m/분 으로 한다.
            self._telemetry["belt"] = round(speed, 3) if self._belt_on else 0.0
            self._telemetry["belt_mpm"] = round(mpm, 1) if self._belt_on else 0.0
            self._telemetry["belt_on"] = self._belt_on

    def on_reset_done(self, level: str = config.RESET_DEFAULT) -> None:
        """리셋 후 그리퍼를 열린 상태로 되돌려 UI와 실제 상태를 맞춘다.

        어떤 강도로 몇 번째 리셋이 끝났는지도 텔레메트리에 남긴다. 요청을 보낸
        쪽(HTTP·ROS)이 **정말 초기화됐는지** 를 확인할 수단이 있어야 한다 —
        없으면 리셋이 먹었는지 알 길이 없어 결국 프로세스를 죽이게 된다.
        """
        with self._lock:
            self._gripper = config.GRIPPER_OPEN
            for client in self._clients.values():
                client.pressed.clear()
                client.held_steps.clear()
            self._telemetry["last_reset"] = level
            self._telemetry["reset_count"] = self._telemetry.get("reset_count", 0) + 1
