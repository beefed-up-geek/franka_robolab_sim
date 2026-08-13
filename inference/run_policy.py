#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""학습한 VLA 로 시뮬레이션을 구동한다 (컨테이너 안, Isaac 번들 ROS).

관측을 모아 호스트의 추론 서버(policy_server.py)에 보내고, 받은 액션을
그대로 /franka/cmd/eef_delta·gripper 로 발행한다. 사람이 브라우저에서
하던 조작을 정책이 대신하는 것이라, 환경은 명령의 출처를 구분하지 않는다.

성공 판정은 태스크마다 시뮬레이션이 쏘는 이벤트를 쓴다:
    task1  tool_crossed      공구가 경계를 넘음
    task2  charging_done     두 커넥터 모두 부착
    task3  can_binned        캔을 통에 넣음 (없으면 이벤트 로그만)

실행 (inference/run.sh 가 감싼다):
    /isaac-sim/python.sh inference/run_policy.py --task task2 --episodes 5
"""
from __future__ import annotations

import argparse
import base64
import json
import sys
import threading
import time
import urllib.request

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

CAMS = ("front", "top", "wrist")
VERTICAL_QUAT = (0.7071068, 0.0, 0.7071068, 0.0)   # 그리퍼가 수직 아래
ABS_STEP_LIMIT = 0.08   # abs 모드에서 한 스텝에 허용하는 최대 이동 [m]
# 한 에피소드의 성공 기준. task2 는 두 커넥터를 모두 꽂아야 성공이다
# (charging_done) — 빨간 커넥터가 붙으면 지시문을 검은 커넥터로 바꿔 이어간다.
# 학습 데이터는 커넥터 하나가 한 에피소드지만, 지시문 전환 + 서버 리셋으로
# 두 에피소드를 이어 붙이는 것과 같은 조건을 만든다.
# task3 은 이벤트가 없다 — 수집기와 같은 방식으로 /franka/status 의 binned
# 카운터 증가(캔이 실제로 통에 들어감)로 판정한다.
SUCCESS_EVENT = {"task1": "tool_crossed", "task2": "charging_done",
                 "task3": None}
TASK_TEXT = {
    "task1": "pass the hammer",
    "task2": "Plug the red charging connector into the battery positive terminal",
    "task3": "Pick up the cans from the conveyor and put them in the bin",
}
TASK2_TEXT_BLACK = "Plug the black charging connector into the battery negative terminal"
# task3 배치(정적) 모드 — 벨트는 수집과 동일하게 정지 상태로 평가한다.
# 성공 판정은 binned 카운터 증가(캔이 통에 들어감)이고, test 환경에서는
# 정상 캔을 모두 담으면 시뮬레이션이 trio_done 을 쏘고 스스로 전체 초기화한다
# (카운터가 0 으로 돌아가므로 아래 루프가 기준선을 되잡는다).
TASK3_BELT_MPM = 0.0


class Bridge(Node):
    def __init__(self, action_mode: str = "delta") -> None:
        super().__init__("vla_runner")
        self.action_mode = action_mode
        self.eef = None
        self.eef_quat = None
        self.gripper = 0.0
        self.images: dict[str, bytes] = {}
        self.events: list[dict] = []
        self.status: dict = {}
        self.create_subscription(PoseStamped, "/franka/eef_pose", self._on_eef, 10)
        self.create_subscription(Float32, "/franka/gripper_state",
                                 lambda m: setattr(self, "gripper", float(m.data)), 10)
        self.create_subscription(String, "/franka/events",
                                 lambda m: self.events.append(json.loads(m.data)), 20)
        self.create_subscription(String, "/franka/status",
                                 lambda m: setattr(self, "status", json.loads(m.data)), 10)
        for cam in CAMS:
            self.create_subscription(
                CompressedImage, f"/franka/camera/{cam}/image_raw/compressed",
                lambda m, c=cam: self.images.__setitem__(c, bytes(m.data)), 2)
        self.pub_delta = self.create_publisher(Twist, "/franka/cmd/eef_delta", 10)
        self.pub_grip = self.create_publisher(Bool, "/franka/cmd/gripper", 10)
        self.pub_reset = self.create_publisher(String, "/franka/cmd/reset", 10)
        self.pub_belt = self.create_publisher(Float32, "/franka/cmd/belt", 10)

    def set_belt_mpm(self, mpm: float) -> None:
        m = Float32()
        m.data = float(mpm)
        self.pub_belt.publish(m)

    def _on_eef(self, m) -> None:
        p, o = m.pose.position, m.pose.orientation
        self.eef = [p.x, p.y, p.z]
        self.eef_quat = [o.w, o.x, o.y, o.z]

    def ready(self) -> bool:
        return self.eef is not None and all(c in self.images for c in CAMS)

    def send(self, action) -> None:
        """정책의 액션을 발행한다 — delta 는 [dx,dy,dz,grip] 그대로,
        abs 는 [x,y,z,grip] 목표 좌표라 현재 EEF 와의 차로 바꿔 발행한다
        (한 스텝 이동은 ABS_STEP_LIMIT 로 제한 — 상태 오차로 목표가 멀어도
        팔이 튀지 않게).

        그리퍼가 수직 아래를 보는 것은 제어기 불변식이라 학습 대상이 아니다 —
        회전 보정은 여기서 현재 자세로부터 직접 계산해 채운다 (수집 때 전문가
        정책이 하던 것과 같은 계산)."""
        if self.action_mode == "abs":
            d = [float(action[i]) - self.eef[i] for i in range(3)]
            n = math.sqrt(sum(c * c for c in d))
            if n > ABS_STEP_LIMIT:
                d = [c * ABS_STEP_LIMIT / n for c in d]
        else:
            d = [float(c) for c in action[0:3]]
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = d
        rot = self.vertical_rot()
        t.angular.x, t.angular.y, t.angular.z = rot
        self.pub_delta.publish(t)
        b = Bool()
        b.data = bool(action[3] > 0.5)
        self.pub_grip.publish(b)

    def halt(self) -> None:
        """정지 — 영(0) delta 발행. abs 모드에서 send([0,0,0,·]) 는 '원점으로
        이동' 이 되므로 에피소드 마감은 반드시 이걸 쓴다."""
        self.pub_delta.publish(Twist())

    def vertical_rot(self, gain: float = 3.0, limit: float = 0.25):
        """수직 아래(VERTICAL_QUAT) 로 되돌리는 각속도 명령."""
        if self.eef_quat is None:
            return (0.0, 0.0, 0.0)
        cw, cx, cy, cz = self.eef_quat
        tw, tx, ty, tz = VERTICAL_QUAT
        iw, ix, iy, iz = cw, -cx, -cy, -cz
        w = tw*iw - tx*ix - ty*iy - tz*iz
        x = tw*ix + tx*iw + ty*iz - tz*iy
        y = tw*iy - tx*iz + ty*iw + tz*ix
        z = tw*iz + tx*iy - ty*ix + tz*iw
        s = math.sqrt(x*x + y*y + z*z)
        if s < 1e-9:
            return (0.0, 0.0, 0.0)
        ang = 2.0 * math.atan2(s, w)
        if ang > math.pi:
            ang -= 2.0 * math.pi
        v = [x / s * ang * gain, y / s * ang * gain, z / s * ang * gain]
        n = math.sqrt(sum(c * c for c in v))
        if n > limit:
            v = [c * limit / n for c in v]
        return tuple(v)

    def request_reset(self, level: str = "full", timeout: float = 30.0) -> bool:
        self.events.clear()
        m = String()
        m.data = level
        self.pub_reset.publish(m)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if any(e.get("type") == "reset_done" for e in self.events):
                self.events.clear()
                return True
            time.sleep(0.05)
        return False


def post(url: str, payload: dict, timeout: float = 20.0) -> dict:
    req = urllib.request.Request(url, data=json.dumps(payload).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def main() -> int:
    ap = argparse.ArgumentParser(description="VLA 추론 구동")
    ap.add_argument("--task", required=True, choices=tuple(SUCCESS_EVENT))
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--rate", type=float, default=6.0)
    ap.add_argument("--timeout", type=float, default=120.0, help="에피소드 제한 [s]")
    ap.add_argument("--server", default="http://127.0.0.1:8010")
    ap.add_argument("--instruction", default=None)
    ap.add_argument("--action-mode", choices=("delta", "abs"), default="delta",
                    help="모델의 액션 표현 — delta 학습 모델은 delta, "
                         "절대좌표 학습 모델은 abs")
    ap.add_argument("--reset-each", action="store_true",
                    help="task3 에서도 에피소드마다 full 초기화한다 (기본은 "
                         "연속 환경). 연속 환경 요인을 분리하는 진단용.")
    args = ap.parse_args()

    rclpy.init()
    node = Bridge(args.action_mode)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        print("[vla] 시뮬레이션 대기 중…", flush=True)
        t0 = time.time()
        while not node.ready() and time.time() - t0 < 30:
            time.sleep(0.1)
        if not node.ready():
            print("[vla] 토픽이 오지 않습니다.", flush=True)
            return 1

        period = 1.0 / args.rate
        ok = 0
        for ep in range(args.episodes):
            # task3 는 에피소드 사이에 환경을 초기화하지 않는다 — 캔을 하나
            # 담을 때마다 장면을 되돌리는 대신 벨트가 이어지는 연속 환경
            # 그대로 다음 에피소드를 센다(실제 라인과 같은 조건). 성공 판정이
            # binned "증가" 라 카운터가 이어져도 문제없다.
            if args.task != "task3" or ep == 0 or args.reset_each:
                node.request_reset("full")
                time.sleep(2.0)
            if args.task == "task3":
                # 배치(정적) 모드 — 수집과 동일하게 벨트 정지.
                node.set_belt_mpm(TASK3_BELT_MPM)
            post(f"{args.server}/reset", {})
            node.events.clear()
            text = args.instruction or TASK_TEXT[args.task]
            binned0 = node.status.get("binned", 0)
            ep_t0 = time.time()
            done = False
            steps = 0
            infer_ms = []
            while not done and time.time() - ep_t0 < args.timeout:
                loop_t = time.time()
                imgs = {c: base64.b64encode(node.images[c]).decode() for c in CAMS}
                res = post(f"{args.server}/act", {
                    "state": node.eef + [node.gripper],
                    "images": imgs, "task": text})
                node.send(res["action"])
                infer_ms.append(res.get("ms", 0.0))
                steps += 1
                if args.task == "task3":
                    _b = node.status.get("binned", 0)
                    if _b < binned0:
                        # test 환경의 라운드 종료(trio_done → full 리셋)로
                        # 카운터가 0 으로 돌아갔다 — 기준선을 되잡는다.
                        binned0 = 0
                    if _b > binned0:
                        done = True
                for e in list(node.events):
                    if SUCCESS_EVENT[args.task] and e.get("type") == SUCCESS_EVENT[args.task]:
                        done = True
                    # task3 test: 정상 캔을 모두 담아 라운드가 끝난 것도 성공이다
                    # (마지막 캔의 binned 증가를 리셋이 삼킨 경우를 덮는다).
                    if args.task == "task3" and e.get("type") == "trio_done":
                        done = True
                    # task2: 빨간 커넥터가 붙으면 검은 커넥터 지시로 전환.
                    # 서버 액션 큐에 남은 빨간 커넥터 청크는 리셋으로 비운다.
                    if (args.task == "task2" and not args.instruction
                            and e.get("type") == "connector_attached"
                            and e.get("connector") == "connector_red"
                            and text != TASK2_TEXT_BLACK):
                        text = TASK2_TEXT_BLACK
                        post(f"{args.server}/reset", {})
                        print("[vla] 빨간 커넥터 부착 — 검은 커넥터로 전환",
                              flush=True)
                node.events.clear()
                sleep = period - (time.time() - loop_t)
                if sleep > 0:
                    time.sleep(sleep)
            node.halt()
            ok += int(done)
            avg = sum(infer_ms) / max(len(infer_ms), 1)
            print(f"[vla] ep{ep + 1}: {'성공' if done else '실패'} "
                  f"({steps}스텝, {time.time() - ep_t0:.0f}s, 추론 {avg:.0f}ms) "
                  f"— 누적 {ok}/{ep + 1}", flush=True)
        print(f"[vla] 결과: {ok}/{args.episodes} 성공 "
              f"({100.0 * ok / max(args.episodes, 1):.0f}%)", flush=True)
        return 0
    finally:
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)


if __name__ == "__main__":
    sys.exit(main())
