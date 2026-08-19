# SPDX-License-Identifier: Apache-2.0
"""VLA_lang 조향(steering) 평가 — **좌표로 지시한 캔을 정말 집는가**.

성공률(run_policy)과는 다른 것을 잰다. Steerable Policies (arXiv:2602.13193)
의 핵심 주장은 "정책이 좌표 지시를 따른다" 이므로, 여기서는 매 에피소드마다
벨트 위 캔 **하나를 좌표로 지목**하고 정책이 그 캔을 집는지 본다.

    지시:  "grasp the can at [0.52, -0.20]"   (지목한 캔의 현재 좌표)
    판정:  처음으로 접촉력 >0.3N 이 잡힌 캔 == 지목한 캔인가 (steer 적중)
           이후 지시를 "put the can in the bin at [0.26, 0.58]" 로 바꿔
           통 투입까지 이어지는지도 본다 (task 성공)

지목은 **일부러 로봇에서 가장 먼 캔**으로 한다. 아무 캔이나 집어도 1/3 은
맞으므로, 임의 선택 정책과 구별하려면 자연스러운 선택(가까운 캔)과 다른
것을 시켜야 한다. 벨트가 저속으로 흐르므로 좌표는 매 스텝 현재 위치로
갱신한다 — 상위 정책(VLM)이 주기적으로 재지시하는 논문 구성과 같다.

비교 기준: 같은 명령을 task3_abs_v10(비조향 학습) 모델에 주면 지시를 이해할
근거가 없으므로 적중률이 우연(≈33%) 근처여야 한다.

사용:
    python steer_eval.py --episodes 10          # task3_train 시뮬 + 서버 필요
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, PoseArray

CAMS = ("front", "top", "wrist")
BIN_XY = (0.26, 0.58)
BASE_XY = (0.0, 0.0)          # 로봇 베이스 — "가장 먼 캔" 의 기준
GRASP_N = 0.3
ABS_STEP_LIMIT = 0.08         # run_policy 와 동일
VERTICAL_QUAT = (0.7071068, 0.0, 0.7071068, 0.0)   # 그리퍼 수직 아래


class Bridge(Node):
    def __init__(self):
        super().__init__("steer_eval")
        self.eef = None
        self.eef_quat = None
        self.gripper = 0.0
        self.images: dict[str, bytes] = {}
        self.names: list[str] = []
        self.objects: list[list[float]] = []
        self.status: dict = {}
        self.events: list[dict] = []
        from geometry_msgs.msg import PoseStamped
        self.create_subscription(PoseStamped, "/franka/eef_pose", self._on_eef, 10)
        self.create_subscription(Float32, "/franka/gripper_state",
                                 lambda m: setattr(self, "gripper", float(m.data)), 10)
        # object_names 는 **latched**(TRANSIENT_LOCAL) 발행이다 — 보통 QoS 로
        # 구독하면 늦게 붙은 노드는 영영 못 받는다 (실측: 8초 대기에도 무소식).
        from rclpy.qos import QoSProfile, DurabilityPolicy
        _latched = QoSProfile(depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/franka/object_names",
                                 lambda m: setattr(self, "names", json.loads(m.data)),
                                 _latched)
        self.create_subscription(PoseArray, "/franka/objects",
                                 lambda m: setattr(self, "objects",
                                                   [[p.position.x, p.position.y, p.position.z]
                                                    for p in m.poses]), 10)
        self.create_subscription(String, "/franka/status",
                                 lambda m: setattr(self, "status", json.loads(m.data)), 10)
        self.create_subscription(String, "/franka/events",
                                 lambda m: self.events.append(json.loads(m.data)), 20)
        for cam in CAMS:
            from sensor_msgs.msg import CompressedImage
            self.create_subscription(
                CompressedImage, f"/franka/camera/{cam}/image_raw/compressed",
                (lambda c: lambda m: self.images.__setitem__(c, bytes(m.data)))(cam), 10)
        self.pub_delta = self.create_publisher(Twist, "/franka/cmd/eef_delta", 10)
        self.pub_grip = self.create_publisher(Bool, "/franka/cmd/gripper", 10)
        self.pub_reset = self.create_publisher(String, "/franka/cmd/reset", 10)
        self.pub_belt = self.create_publisher(Float32, "/franka/cmd/belt", 10)

    def _on_eef(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.eef = [p.x, p.y, p.z]
        self.eef_quat = [o.w, o.x, o.y, o.z]

    def vertical_rot(self, gain: float = 3.0, limit: float = 0.25):
        """수직 아래(VERTICAL_QUAT)로 되돌리는 각속도 — run_policy 와 동일.

        그리퍼가 수직을 보는 것은 제어기 불변식이라 학습 대상이 아니다.
        이 보정이 빠지면 에피소드가 길어질수록 자세가 흘러 파지가 무너진다.
        """
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

    def cans(self) -> dict[str, list[float]]:
        """벨트 위(active) 캔 이름 → 위치."""
        active = set(self.status.get("active") or [])
        out = {}
        for n, p in zip(self.names, self.objects):
            if n in active:
                out[n] = p
        return out

    def ready(self) -> bool:
        return (self.eef is not None and self.names
                and all(c in self.images for c in CAMS))

    def send_abs(self, action):
        """abs 모델 출력 → 상대 명령 (run_policy 의 abs 변환과 동일).

        수직 자세 보정(vertical_rot)까지 포함해야 한다 — 없으면 자세가 흐른다.
        """
        d = [float(action[i]) - self.eef[i] for i in range(3)]
        n = math.sqrt(sum(v * v for v in d))
        if n > ABS_STEP_LIMIT:
            d = [v * ABS_STEP_LIMIT / n for v in d]
        m = Twist()
        m.linear.x, m.linear.y, m.linear.z = d
        m.angular.x, m.angular.y, m.angular.z = self.vertical_rot()
        self.pub_delta.publish(m)
        b = Bool()
        b.data = bool(action[3] > 0.5)
        self.pub_grip.publish(b)

    def halt(self):
        self.pub_delta.publish(Twist())

    def request_reset(self, timeout: float = 30.0) -> bool:
        self.events.clear()
        m = String()
        m.data = "full"
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
    ap = argparse.ArgumentParser(description="좌표 조향 평가")
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--rate", type=float, default=6.0)
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--server", default="http://127.0.0.1:8010")
    ap.add_argument("--pick", choices=("far", "near", "random"), default="far",
                    help="지목할 캔 — far(기본)는 로봇에서 가장 먼 캔이라 "
                         "임의 선택 정책과 가장 잘 구별된다")
    args = ap.parse_args()

    import random
    rclpy.init()
    node = Bridge()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        print("[steer] 시뮬레이션 대기 중…", flush=True)
        t0 = time.time()
        while not node.ready() and time.time() - t0 < 30:
            time.sleep(0.1)
        if not node.ready():
            print("[steer] 토픽이 오지 않습니다.", flush=True)
            return 1

        period = 1.0 / args.rate
        steer_ok = 0
        task_ok = 0
        for ep in range(args.episodes):
            node.request_reset()
            time.sleep(2.0)
            post(f"{args.server}/reset", {})
            node.events.clear()
            cans = node.cans()
            if len(cans) < 2:
                time.sleep(2.0)
                cans = node.cans()
            # 지목 — 기본은 가장 먼 캔
            if args.pick == "far":
                cmd_can = max(cans, key=lambda n: math.dist(cans[n][:2], BASE_XY))
            elif args.pick == "near":
                cmd_can = min(cans, key=lambda n: math.dist(cans[n][:2], BASE_XY))
            else:
                cmd_can = random.choice(sorted(cans))
            print(f"[steer] ep{ep+1}: 지목 {cmd_can} @ "
                  f"{[round(v, 2) for v in cans[cmd_can][:2]]} "
                  f"(후보 {sorted(cans)})", flush=True)

            grasped = None       # 처음 물린 캔
            binned = False
            ep_t0 = time.time()
            while time.time() - ep_t0 < args.timeout:
                loop_t = time.time()
                cans_now = node.cans()
                pos = cans_now.get(cmd_can)
                if grasped is None:
                    if pos is None:
                        break                      # 지목 캔이 사라짐 (담겼거나 낙하)
                    text = f"grasp the can at [{pos[0]:.2f}, {pos[1]:.2f}]"
                else:
                    text = f"put the can in the bin at [{BIN_XY[0]:.2f}, {BIN_XY[1]:.2f}]"
                imgs = {c: base64.b64encode(node.images[c]).decode() for c in CAMS}
                res = post(f"{args.server}/act", {
                    "state": node.eef + [node.gripper],
                    "images": imgs, "task": text})
                node.send_abs(res["action"])
                # 어느 캔이 물렸는가 — 객체별 필터 접촉력
                if grasped is None:
                    contact = node.status.get("contact") or {}
                    for n in cans_now:
                        if float(contact.get(n, 0.0)) > GRASP_N:
                            grasped = n
                            print(f"[steer]   파지 감지: {n} "
                                  f"{'✓ 지목대로' if n == cmd_can else '✗ 다른 캔'}",
                                  flush=True)
                            break
                for e in list(node.events):
                    if e.get("type") in ("trio_spawn", "trio_done"):
                        binned = True
                node.events.clear()
                if grasped is not None and cmd_can not in node.cans():
                    binned = True                 # 지목 캔이 벨트에서 사라짐
                if binned:
                    break
                sleep = period - (time.time() - loop_t)
                if sleep > 0:
                    time.sleep(sleep)
            node.halt()
            hit = grasped == cmd_can
            steer_ok += int(hit)
            task_ok += int(hit and binned)
            print(f"[steer] ep{ep+1}: steer {'적중' if hit else '빗나감'}"
                  f" (물린 캔 {grasped}) · 통 {'투입' if binned else '미투입'}"
                  f" — 누적 steer {steer_ok}/{ep+1} · task {task_ok}/{ep+1}",
                  flush=True)
        print(f"[steer] 결과: steer {steer_ok}/{args.episodes} "
              f"({100.0*steer_ok/max(args.episodes,1):.0f}%) · "
              f"task {task_ok}/{args.episodes}", flush=True)
        return 0
    finally:
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)


if __name__ == "__main__":
    sys.exit(main())
