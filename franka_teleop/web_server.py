# SPDX-License-Identifier: Apache-2.0
"""브라우저 ↔ 시뮬레이션 브리지.

단일 포트(8003)에서 세 가지를 모두 제공한다. 서버 앞단 방화벽이 8003 하나만
열어주기 때문에 포트를 나눌 수 없다.

  GET /        조작 UI (HTML)
  GET /stream  카메라 영상 (multipart MJPEG)
  GET /ws      키/마우스 입력 업링크 + 텔레메트리 다운링크 (WebSocket)

Isaac Sim이 메인 스레드를 점유하므로 이 서버는 별도 스레드에서 자체 asyncio
이벤트 루프를 돌린다. 공유 지점은 TeleopState 하나뿐이다.
"""
from __future__ import annotations

import asyncio
import itertools
import json
import logging
import threading
from pathlib import Path

from aiohttp import WSMsgType, web

from franka_teleop import config
from franka_teleop.state import TeleopState

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"

# 연결마다 고유 id — 키 눌림 상태와 하트비트를 클라이언트별로 관리하기 위한 것이다.
_next_client_id = itertools.count(1)


async def _index(request: web.Request) -> web.Response:
    return web.FileResponse(STATIC_DIR / "index.html")


async def _stream(request: web.Request) -> web.StreamResponse:
    """MJPEG 스트림 — 새 프레임이 올라올 때만 내보낸다."""
    state: TeleopState = request.app["state"]
    response = web.StreamResponse(
        status=200,
        headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store, no-cache, must-revalidate",
        },
    )
    await response.prepare(request)

    last_id = -1
    interval = 1.0 / config.STREAM_MAX_FPS
    try:
        while True:
            frame, frame_id = state.get_frame()
            if frame is not None and frame_id != last_id:
                last_id = frame_id
                await response.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
                )
            await asyncio.sleep(interval)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return response


async def _ws(request: web.Request) -> web.WebSocketResponse:
    state: TeleopState = request.app["state"]
    ws = web.WebSocketResponse(heartbeat=10.0)
    await ws.prepare(request)
    client_id = next(_next_client_id)
    state.add_client(client_id)
    logger.info("클라이언트 접속 #%d (현재 %d)", client_id, state.client_count)

    async def push_telemetry() -> None:
        """10Hz로 EEF 좌표·그리퍼 상태를 UI에 밀어준다."""
        try:
            while not ws.closed:
                await ws.send_str(json.dumps({"type": "telemetry", **state.get_telemetry()}))
                await asyncio.sleep(0.1)
        except (ConnectionResetError, asyncio.CancelledError):
            pass

    pusher = asyncio.create_task(push_telemetry())
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                continue

            kind = data.get("type")
            if kind == "key":
                state.on_key(client_id, data.get("code", ""), bool(data.get("down")))
            elif kind == "orbit":
                state.orbit(float(data.get("dx", 0.0)), float(data.get("dy", 0.0)))
            elif kind == "zoom":
                state.zoom(float(data.get("d", 0.0)))
            elif kind == "blur":
                # 브라우저 탭이 포커스를 잃으면 keyup이 안 오므로 즉시 정지시킨다.
                state.release_all(client_id)
            elif kind == "ping":
                state.heartbeat(client_id)
    finally:
        pusher.cancel()
        # 연결이 끊기면 그 클라이언트가 누르고 있던 키도 같이 사라진다.
        state.remove_client(client_id)
        logger.info("클라이언트 종료 #%d (현재 %d)", client_id, state.client_count)
    return ws


def _build_app(state: TeleopState) -> web.Application:
    app = web.Application()
    app["state"] = state
    app.router.add_get("/", _index)
    app.router.add_get("/stream", _stream)
    app.router.add_get("/ws", _ws)
    return app


def start_in_thread(state: TeleopState) -> threading.Thread:
    """웹 서버를 데몬 스레드로 띄우고 즉시 반환한다."""

    def _run() -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        runner = web.AppRunner(_build_app(state), access_log=None)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, config.HOST, config.PORT)
        loop.run_until_complete(site.start())
        logger.info("텔레오퍼레이션 서버 기동: http://%s:%d", config.HOST, config.PORT)
        loop.run_forever()

    thread = threading.Thread(target=_run, name="teleop-web", daemon=True)
    thread.start()
    return thread
