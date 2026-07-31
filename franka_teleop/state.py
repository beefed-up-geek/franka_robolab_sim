# SPDX-License-Identifier: Apache-2.0
"""시뮬레이션 스레드와 웹 서버 스레드가 공유하는 상태.

Isaac Sim(Omniverse Kit)은 반드시 메인 스레드에서 돌아야 하므로 aiohttp 서버를
별도 스레드에 띄운다. 두 스레드가 만나는 지점은 이 클래스 하나뿐이며, 모든
접근은 단일 Lock으로 직렬화한다.

  웹 스레드 → on_key()/request_reset()   (입력)
  심 스레드 → consume()/publish_frame()  (제어 + 영상)
"""
from __future__ import annotations

import threading
import time

from franka_teleop import config


class TeleopState:
    def __init__(self) -> None:
        self._lock = threading.Lock()

        # 입력
        self._pressed: set[str] = set()
        self._held_steps: dict[str, int] = {}
        self._last_input_t = time.monotonic()

        # 1회성 이벤트
        self._gripper = config.GRIPPER_OPEN
        self._reset_requested = False
        self._speed_idx = config.SPEED_DEFAULT_INDEX

        # 영상
        self._frame: bytes | None = None
        self._frame_id = 0

        # UI 표시용 텔레메트리
        self._telemetry: dict = {}

        self.clients = 0

    # ── 웹 스레드 ────────────────────────────────────────────────────────
    def on_key(self, code: str, down: bool) -> None:
        with self._lock:
            self._last_input_t = time.monotonic()
            if code == config.KEY_GRIPPER_TOGGLE:
                # 토글은 눌림 시작에만 반응 — 누르고 있어도 한 번만 뒤집힌다.
                if down and code not in self._pressed:
                    self._gripper = (
                        config.GRIPPER_CLOSE
                        if self._gripper == config.GRIPPER_OPEN
                        else config.GRIPPER_OPEN
                    )
            elif code == config.KEY_RESET:
                if down and code not in self._pressed:
                    self._reset_requested = True
            elif code in (config.KEY_SPEED_DOWN, config.KEY_SPEED_UP):
                if down and code not in self._pressed:
                    step = -1 if code == config.KEY_SPEED_DOWN else +1
                    self._speed_idx = max(0, min(len(config.SPEED_LEVELS) - 1, self._speed_idx + step))

            if down:
                self._pressed.add(code)
            else:
                self._pressed.discard(code)
                self._held_steps.pop(code, None)

    def release_all(self) -> None:
        """브라우저 연결이 끊기거나 창이 포커스를 잃었을 때 즉시 정지."""
        with self._lock:
            self._pressed.clear()
            self._held_steps.clear()

    def heartbeat(self) -> None:
        with self._lock:
            self._last_input_t = time.monotonic()

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
            # 입력이 끊긴 지 오래면 폭주 방지를 위해 전부 해제한다.
            if time.monotonic() - self._last_input_t > config.INPUT_TIMEOUT_S:
                self._pressed.clear()
                self._held_steps.clear()

            delta = [0.0] * 6
            for code in self._pressed:
                mapping = config.KEY_MAP.get(code)
                if mapping is None:
                    continue
                idx, sign = mapping

                # 누르고 있는 동안 0 → 1 로 서서히 올려 급출발을 막는다.
                held = self._held_steps.get(code, 0) + 1
                self._held_steps[code] = held
                ramp = min(1.0, held / max(1, config.RAMP_STEPS))

                scale = config.POS_DELTA if idx < 3 else config.ROT_DELTA
                delta[idx] += sign * scale * ramp * config.SPEED_LEVELS[self._speed_idx]

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
            self._telemetry["gripper"] = "CLOSED" if self._gripper == config.GRIPPER_CLOSE else "OPEN"
            self._telemetry["speed"] = config.SPEED_LEVELS[self._speed_idx]

    def on_reset_done(self) -> None:
        """리셋 후 그리퍼를 열린 상태로 되돌려 UI와 실제 상태를 맞춘다."""
        with self._lock:
            self._gripper = config.GRIPPER_OPEN
            self._pressed.clear()
            self._held_steps.clear()
