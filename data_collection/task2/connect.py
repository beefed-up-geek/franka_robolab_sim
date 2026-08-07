#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""task2 연결 드라이버 — 빨강→B(+), 검정→A(-) 순서로 커넥터를 단자에 씌운다.

시뮬레이션과는 ROS 로만 이어진다 (task1 deliver 와 같은 구조).
부착 판정은 시뮬레이션(runner)이 하고, 여기서는 connector_attached 이벤트를
보고 다음 단계로 넘어간다. 둘 다 부착되면 시뮬레이션이 charging_done 을
쏘면서 커넥터를 초기화한다 — 그것이 성공 판정이다.

    ./data_collection/task2/run.sh            # 기본: red -> black 순서
"""
from __future__ import annotations

import argparse
import random
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy                                            # noqa: E402
from geometry_msgs.msg import PoseArray, PoseStamped, Twist   # noqa: E402
from rclpy.node import Node                             # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from std_msgs.msg import Bool, Float32, String          # noqa: E402

from policy import Task2ConnectPolicy                   # noqa: E402

# 배터리 로컬 단자 오프셋 (SAM3D 정점 측정) — runner 의 _TERMS 와 같은 근거.
# 씬에서 배터리가 z180 으로 놓이므로 월드 = 배터리위치 + (-lx, -ly, +lz).
TERMS_LOCAL = {
    "connector_red": (0.0962, -0.0546, 0.208),      # B(+) 단자 — 접촉 증거 교정
    "connector_black": (-0.0962, -0.0523, 0.208),   # A(-) 단자 — 대칭 추정 (접촉으로 재교정 예정)
}
HOLD_Z = 0.05      # 커넥터가 이만큼 들려 있으면 "물었다" (접촉력이 약해 깜빡이는 오탐 보완)


class Bridge(Node):
    def __init__(self) -> None:
        super().__init__("task2_connect")
        self.eef = None
        self.eef_quat = None
        self.names = []
        self.objects = []
        self.status = {}
        self.events = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, "/franka/eef_pose", self._on_eef, 10)
        self.create_subscription(PoseArray, "/franka/objects",
                                 lambda m: setattr(self, "objects",
                                                   [[p.position.x, p.position.y, p.position.z]
                                                    for p in m.poses]), 10)
        self.create_subscription(String, "/franka/grasp_info",
                                 lambda m: setattr(self, "ginfo", json.loads(m.data)), 10)
        self.create_subscription(String, "/franka/status",
                                 lambda m: setattr(self, "status", json.loads(m.data)), 10)
        self.create_subscription(String, "/franka/object_names",
                                 lambda m: setattr(self, "names", json.loads(m.data)), latched)
        self.create_subscription(String, "/franka/events",
                                 lambda m: self.events.append(json.loads(m.data)), 20)
        self.pub_delta = self.create_publisher(Twist, "/franka/cmd/eef_delta", 10)
        self.pub_grip = self.create_publisher(Bool, "/franka/cmd/gripper", 10)
        self.pub_reset = self.create_publisher(String, "/franka/cmd/reset", 10)

    def _on_eef(self, m) -> None:
        p, o = m.pose.position, m.pose.orientation
        self.eef = [p.x, p.y, p.z]
        self.eef_quat = [o.w, o.x, o.y, o.z]

    def tools(self):
        return [{"name": n, "pos": p} for n, p in zip(self.names, self.objects)]

    def flange_offset(self) -> float:
        return 0.145

    def send(self, delta, close) -> None:
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = \
            float(delta[0]), float(delta[1]), float(delta[2])
        t.angular.x, t.angular.y, t.angular.z = \
            float(delta[3]), float(delta[4]), float(delta[5])
        self.pub_delta.publish(t)
        b = Bool()
        b.data = bool(close)
        self.pub_grip.publish(b)

    def ready(self) -> bool:
        return self.eef is not None and self.names and self.objects

    def request_reset(self, level: str, timeout: float = 30.0) -> bool:
        self.events.clear()
        m = String()
        m.data = level
        self.pub_reset.publish(m)
        t0 = time.time()
        while time.time() - t0 < timeout:
            for e in list(self.events):
                if e.get("type") == "reset_done" and e.get("source") == "request":
                    self.events.clear()
                    return True
            time.sleep(0.05)
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description="task2 커넥터 연결")
    ap.add_argument("--rate", type=float, default=6.0)
    ap.add_argument("--reset", choices=("none", "soft", "hard", "full"), default="full")
    ap.add_argument("--order", type=str, default="red,black",
                    help="연결 순서 (실무 규칙: + 먼저)")
    ap.add_argument("--seed", type=int, default=None,
                    help="궤적·속도 샘플링 시드 (기본: 시간 기반)")
    ap.add_argument("--speed", type=float, default=None,
                    help="속도 배수 고정 (기본: 0.9~1.5 샘플)")
    ap.add_argument("--traj", type=int, default=None,
                    help="궤적 가족 고정 0~13 (기본: 샘플)")
    args = ap.parse_args()

    rclpy.init()
    node = Bridge()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        return _run(node, args)
    finally:
        # rclpy.spin 데몬 스레드가 살아있는 채로 인터프리터가 내려가면 DDS
        # 소멸자 경합으로 코어 덤프가 난다 (terminate called ... — 실측).
        # 컨텍스트를 먼저 닫아 spin 을 끝내고 스레드를 거둔 뒤 나간다.
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)


def _run(node, args) -> int:
    print("[connect] 시뮬레이션 대기 중…", flush=True)
    t0 = time.time()
    while not node.ready() and time.time() - t0 < 30:
        time.sleep(0.1)
    if not node.ready():
        print("[connect] 토픽이 오지 않습니다.", flush=True)
        return 1
    if args.reset != "none":
        print(f"[connect] 초기화({args.reset}) 요청…", flush=True)
        node.request_reset(args.reset)
        time.sleep(2.0)

    seed = args.seed if args.seed is not None else int(time.time()) % 1000000
    print(f"[connect] 시드 {seed}", flush=True)
    t_all = time.time()

    period = 1.0 / args.rate
    seq = [f"connector_{c.strip()}" for c in args.order.split(",")]

    for name in seq:
      for attempt in range(3):
        bat = next((t for t in node.tools() if t["name"] == "battery"), None)
        if bat is None:
            print("[connect] 배터리 관측 없음", flush=True)
            return 1
        # 시뮬레이션이 ROS status 로 단자 월드 좌표를 발행한다 — 그것을 쓴다.
        # (없을 때만 로컬 상수로 계산하는 예비 경로)
        key = "pos" if name == "connector_red" else "neg"
        pub = (node.status.get("terminals") or {}).get(key)
        if pub:
            term = list(pub)
        else:
            lx, ly, lz = TERMS_LOCAL[name]
            term = [bat["pos"][0] - lx, bat["pos"][1] - ly, bat["pos"][2] + lz]
        pol = "B(+)" if name == "connector_red" else "A(-)"
        rng = random.Random(seed * 10000 + attempt * 100
                            + (1 if name == "connector_red" else 2))
        policy = Task2ConnectPolicy(name, term, rng=rng,
                                    speed=args.speed, family=args.traj)
        print(f"[connect] {name} → {pol} 단자 {['%.3f' % v for v in term]} "
              f"| {policy.describe()}", flush=True)
        node.events.clear()
        attached = False
        t0 = time.time()
        _prev_stage, _last_dbg = None, 0.0
        while time.time() - t0 < 180:
            loop_t = time.time()
            st = node.status
            tool = next((t for t in node.tools() if t["name"] == name), None)
            contact = float((st.get("contact") or {}).get(name, 0.0))
            lifted = bool(tool and tool["pos"][2] > HOLD_Z)
            delta, close, info = policy.act(
                node.eef, node.eef_quat, node.tools(), node.flange_offset(),
                gripping=contact > 0.12 or lifted)
            node.send(delta, close)
            if info.get("stage") != _prev_stage:
                print(f"    [{info.get('stage')}] eef={[round(v,3) for v in node.eef]}", flush=True)
                _prev_stage = info.get("stage")
            if time.time() - _last_dbg > 3.0:
                _last_dbg = time.time()
                cp = tool["pos"] if tool else None
                print(f"    dbg {info.get('stage')} err={info.get('err')} "
                      f"eef={[round(v,3) for v in node.eef]} "
                      f"conn={[round(v,3) for v in cp] if cp else None} 접촉={contact:.1f}", flush=True)
            for e in list(node.events):
                if e.get("type") == "connector_attached" and e.get("connector") == name:
                    attached = True
            if attached:
                break
            if info.get("abort"):
                print(f"[connect] 중단 — {info.get('why')}", flush=True)
                break
            sleep = period - (time.time() - loop_t)
            if sleep > 0:
                time.sleep(sleep)
        if not attached:
            print(f"[connect] 시도 {attempt+1} 실패 — {name} (단계 {policy.stage})", flush=True)
            node.send([0.0] * 6, False)
            if attempt < 2:
                node.request_reset("full")
                time.sleep(2.0)
                continue
            return 1
        print(f"[connect] 부착 확인 — {name}", flush=True)
        # 손을 놓고 위로 물러난다
        for _ in range(10):
            node.send([0.0, 0.0, 0.06, 0.0, 0.0, 0.0], False)
            time.sleep(period)
        break

    # 둘 다 부착 → 시뮬레이션의 완료 초기화(charging_done)를 기다린다
    t0 = time.time()
    while time.time() - t0 < 20:
        if any(e.get("type") == "charging_done" for e in node.events):
            print(f"[connect] 성공 — 충전 연결 완료, 커넥터 자동 초기화됨 "
                  f"(총 {time.time() - t_all:.0f}s)", flush=True)
            node.send([0.0] * 6, False)
            return 0
        time.sleep(0.2)
    print("[connect] 경고 — 부착 2건 후 완료 이벤트 없음", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
