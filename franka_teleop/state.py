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

from franka_teleop import config


@dataclass
class _Client:
    """한 브라우저 연결의 입력 상태."""
    pressed: set[str] = field(default_factory=set)
    held_steps: dict[str, int] = field(default_factory=dict)
    last_input: float = field(default_factory=time.monotonic)

    def stale(self) -> bool:
        return time.monotonic() - self.last_input > config.INPUT_TIMEOUT_S


class TeleopState:
    def __init__(self) -> None:
        self._lock = threading.Lock()

        self._clients: dict[int, _Client] = {}

        # 1회성 이벤트 / 전역 설정 — 여러 클라이언트가 공유해도 무방하다.
        self._gripper = config.GRIPPER_OPEN
        self._reset_requested = False
        self._speed_idx = config.SPEED_DEFAULT_INDEX

        # 카메라 궤도 — CAM_TARGET 을 중심으로 한 구면좌표
        self._cam_az = config.CAM_AZIMUTH
        self._cam_el = config.CAM_ELEVATION
        self._cam_radius = config.CAM_RADIUS

        # 영상
        self._frame: bytes | None = None
        self._frame_id = 0

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
                    self._reset_requested = True
            elif code == config.KEY_VIEW_RESET:
                if down and code not in client.pressed:
                    self._cam_az = config.CAM_AZIMUTH
                    self._cam_el = config.CAM_ELEVATION
                    self._cam_radius = config.CAM_RADIUS
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

    def camera_azimuth(self) -> float:
        with self._lock:
            return self._cam_az

    def get_frame(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._frame, self._frame_id

    def get_telemetry(self) -> dict:
        with self._lock:
            return dict(self._telemetry)

    # ── 심 스레드 ────────────────────────────────────────────────────────
    def consume(self) -> tuple[list[float], float, bool]:
        """이번 스텝의 (6축 델타, 그리퍼, 리셋요청)을 계산해 돌려준다."""
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

            reset = self._reset_requested
            self._reset_requested = False
            return delta, self._gripper, reset

    def publish_frame(self, jpeg: bytes) -> None:
        with self._lock:
            self._frame = jpeg
            self._frame_id += 1

    def publish_telemetry(self, **kwargs) -> None:
        with self._lock:
            self._telemetry.update(kwargs)
            self._telemetry["gripper"] = (
                "CLOSED" if self._gripper == config.GRIPPER_CLOSE else "OPEN"
            )
            self._telemetry["speed"] = config.SPEED_LEVELS[self._speed_idx]
            self._telemetry["cam_radius"] = round(self._cam_radius, 2)

    def on_reset_done(self) -> None:
        """리셋 후 그리퍼를 열린 상태로 되돌려 UI와 실제 상태를 맞춘다."""
        with self._lock:
            self._gripper = config.GRIPPER_OPEN
            for client in self._clients.values():
                client.pressed.clear()
                client.held_steps.clear()
