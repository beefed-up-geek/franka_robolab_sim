# SPDX-License-Identifier: Apache-2.0
"""브라우저 ↔ 시뮬레이션 브리지.

단일 포트(8003)에서 세 가지를 모두 제공한다. 서버 앞단 방화벽이 8003 하나만
열어주기 때문에 포트를 나눌 수 없다.

  GET /        조작 UI (HTML)
  GET /stream  카메라 영상 (multipart MJPEG)
  GET /ws      키 입력 업링크 + 텔레메트리 다운링크 (WebSocket)

Isaac Sim이 메인 스레드를 점유하므로 이 서버는 별도 스레드에서 자체 asyncio
이벤트 루프를 돌린다. 공유 지점은 TeleopState 하나뿐이다.
"""
from __future__ import annotations

import asyncio
import json
import logging
import threading
from pathlib import Path

from aiohttp import WSMsgType, web

from franka_teleop import config
from franka_teleop.state import TeleopState

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"


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
    state.clients += 1
    logger.info("클라이언트 접속 (현재 %d)", state.clients)

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
                state.on_key(data.get("code", ""), bool(data.get("down")))
            elif kind == "blur":
                # 브라우저 탭이 포커스를 잃으면 keyup이 안 오므로 즉시 정지시킨다.
                state.release_all()
            elif kind == "ping":
                state.heartbeat()
    finally:
        pusher.cancel()
        state.clients -= 1
        # 마지막 클라이언트가 나가면 로봇을 멈춘다.
        if state.clients <= 0:
            state.release_all()
        logger.info("클라이언트 종료 (현재 %d)", state.clients)
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
