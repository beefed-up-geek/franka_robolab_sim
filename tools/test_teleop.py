"""브라우저 없이 텔레오퍼레이션 동작을 검증한다.

  python3 tools/test_teleop.py --host <서버주소>

6축 이동 방향, 그리퍼 토글, 안전장치(blur/입력 타임아웃)를 확인한다.
websockets 패키지가 필요하다 (pip install websockets).


서버는 10Hz 로 텔레메트리를 밀어주므로, 수신을 게을리하면 큐에 쌓인 옛 값을 읽게
된다. 그래서 수신 전용 태스크를 따로 돌려 항상 "가장 최근" 상태만 본다
(브라우저의 onmessage 와 같은 동작).
"""
import argparse
import asyncio
import json

import websockets

parser = argparse.ArgumentParser(description="텔레오퍼레이션 스모크 테스트")
parser.add_argument("--host", default="127.0.0.1", help="텔레오퍼레이션 서버 주소")
parser.add_argument("--port", type=int, default=8003)
args = parser.parse_args()
URL = f"ws://{args.host}:{args.port}/ws"


class Client:
    def __init__(self, ws):
        self.ws = ws
        self.latest = None
        self._task = asyncio.create_task(self._recv_loop())

    async def _recv_loop(self):
        try:
            async for raw in self.ws:
                msg = json.loads(raw)
                if msg.get("type") == "telemetry" and msg.get("ee_x") is not None:
                    self.latest = msg
        except Exception:
            pass

    async def state(self):
        while self.latest is None:
            await asyncio.sleep(0.05)
        return dict(self.latest)

    async def hold(self, code, seconds):
        """키를 눌렀다 떼며 그동안 하트비트를 보낸다 (브라우저와 같은 동작)."""
        await self.ws.send(json.dumps({"type": "key", "code": code, "down": True}))
        end = asyncio.get_event_loop().time() + seconds
        while asyncio.get_event_loop().time() < end:
            await self.ws.send(json.dumps({"type": "ping"}))
            await asyncio.sleep(0.2)
        await self.ws.send(json.dumps({"type": "key", "code": code, "down": False}))
        await asyncio.sleep(0.8)   # 마지막 스텝이 반영될 시간

    async def tap(self, code):
        await self.ws.send(json.dumps({"type": "key", "code": code, "down": True}))
        await asyncio.sleep(0.25)
        await self.ws.send(json.dumps({"type": "key", "code": code, "down": False}))
        await asyncio.sleep(1.2)


async def main():
    async with websockets.connect(URL, max_size=None) as ws:
        cli = Client(ws)
        base = await cli.state()
        print(f"시작 EEF: x={base['ee_x']:.4f} y={base['ee_y']:.4f} z={base['ee_z']:.4f} "
              f"gripper={base['gripper']} 제어={base.get('hz', 0):.1f}Hz\n")

        results = []
        for code, axis, sign, label in [
            ("KeyW", "ee_x", +1, "W → +X"),
            ("KeyS", "ee_x", -1, "S → −X"),
            ("KeyA", "ee_y", +1, "A → +Y"),
            ("KeyD", "ee_y", -1, "D → −Y"),
            ("KeyQ", "ee_z", +1, "Q → +Z"),
            ("KeyE", "ee_z", -1, "E → −Z"),
        ]:
            before = await cli.state()
            await cli.hold(code, 2.0)
            after = await cli.state()
            delta = after[axis] - before[axis]
            ok = (delta * sign) > 0.005
            results.append((label, ok))
            print(f"  {label:9s} Δ{axis}={delta:+.4f} m  {'✅' if ok else '❌'}"
                  f"{'   ' + after['warn'] if after.get('warn') else ''}")

        # 그리퍼 토글 (닫기 → 열기 두 번 모두 확인)
        g0 = (await cli.state())["gripper"]
        await cli.tap("Space")
        g1 = (await cli.state())["gripper"]
        await cli.tap("Space")
        g2 = (await cli.state())["gripper"]
        grip_ok = g0 != g1 and g1 != g2 and g0 == g2
        results.append(("Space 그리퍼", grip_ok))
        print(f"\n  {'Space':9s} {g0} → {g1} → {g2}  {'✅' if grip_ok else '❌'}")

        # 안전장치: 키를 누른 채 blur 를 보내면 즉시 멈춰야 한다
        await ws.send(json.dumps({"type": "key", "code": "KeyW", "down": True}))
        await asyncio.sleep(0.4)
        await ws.send(json.dumps({"type": "blur"}))
        await asyncio.sleep(0.8)
        p1 = await cli.state()
        await asyncio.sleep(2.0)
        p2 = await cli.state()
        drift = abs(p2["ee_x"] - p1["ee_x"])
        blur_ok = drift < 0.004
        results.append(("blur 안전정지", blur_ok))
        print(f"  {'blur':9s} 정지 후 2초 이동량={drift:.4f} m  {'✅' if blur_ok else '❌'}")

        # 안전장치: 키를 누른 채 하트비트를 끊으면 타임아웃으로 멈춰야 한다
        await ws.send(json.dumps({"type": "key", "code": "KeyW", "down": True}))
        await asyncio.sleep(1.2)          # INPUT_TIMEOUT_S(0.5s) 를 넘긴다
        q1 = await cli.state()
        await asyncio.sleep(2.0)
        q2 = await cli.state()
        drift2 = abs(q2["ee_x"] - q1["ee_x"])
        to_ok = drift2 < 0.004
        results.append(("입력 타임아웃", to_ok))
        print(f"  {'timeout':9s} 하트비트 두절 후 2초 이동량={drift2:.4f} m  {'✅' if to_ok else '❌'}")
        await ws.send(json.dumps({"type": "key", "code": "KeyW", "down": False}))

        passed = sum(1 for _, ok in results if ok)
        print(f"\n결과: {passed}/{len(results)} 통과")
        for label, ok in results:
            if not ok:
                print(f"  실패: {label}")
        return passed == len(results)


if __name__ == "__main__":
    raise SystemExit(0 if asyncio.run(main()) else 1)
