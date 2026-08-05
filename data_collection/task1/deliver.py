#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""task1 전달 실행기 — 지정 공구를 노란 테이프 너머로 전달한다.

시뮬레이션과는 ROS 로만 이어진다 (task3 수집기와 같은 구조).

    data_collection/task1/run.sh --tool hammer --yaw 90 --speed 0.2
    data_collection/task1/run.sh --tool drill --yaw 270 --speed 0.35
    data_collection/task1/run.sh --tool scissors

성공 판정: 그리퍼를 연 뒤 공구가 테이프(y=-0.40)를 넘어 작업자 구역 바닥에
있고(z 가 상판 근처), 상판 밖으로 떨어지지 않았을 것.
"""
from __future__ import annotations

import argparse
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
from std_msgs.msg import Bool, String                   # noqa: E402

from policy import Task1DeliverPolicy                   # noqa: E402

TOOL_NAMES = {"hammer": "hammer_7", "drill": "cordless_drill"}
EPISODE_TIMEOUT_S = 90.0


class Bridge(Node):
    def __init__(self) -> None:
        super().__init__("task1_deliver")
        self.eef = None
        self.eef_quat = None
        self.names: list[str] = []
        self.objects: list[list[float]] = []
        self.grasps: list[list[float]] = []
        self.ginfo: list[dict] = []
        self.status: dict = {}
        self.events: list[dict] = []

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, "/franka/eef_pose", self._on_eef, 10)
        self.create_subscription(PoseArray, "/franka/objects",
                                 lambda m: setattr(self, "objects", _xyz(m)), 10)
        self.create_subscription(PoseArray, "/franka/grasp_poses",
                                 lambda m: setattr(self, "grasps", _xyz(m)), 10)
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

    def tools(self) -> list[dict]:
        out = []
        n = min(len(self.names), len(self.objects), len(self.grasps))
        for i in range(n):
            info = next((g for g in self.ginfo if g.get("name") == self.names[i]), {})
            out.append({"name": self.names[i], "pos": self.objects[i],
                        "flange": self.grasps[i],
                        "half_height": float(info.get("half_height", 0.02))})
        return out

    def flange_offset(self) -> float:
        for i in range(min(len(self.objects), len(self.grasps))):
            return self.grasps[i][2] - self.objects[i][2]
        return 0.15

    def send(self, delta, close) -> None:
        t = Twist()
        # rclpy C 확장은 int 를 거부한다(assert PyFloat_Check) — 반드시 float 로.
        t.linear.x, t.linear.y, t.linear.z = (float(delta[0]), float(delta[1]),
                                              float(delta[2]))
        t.angular.x, t.angular.y, t.angular.z = (float(delta[3]), float(delta[4]),
                                                 float(delta[5]))
        self.pub_delta.publish(t)
        b = Bool(); b.data = bool(close)
        self.pub_grip.publish(b)

    def ready(self) -> bool:
        return bool(self.eef is not None and self.names and self.objects and self.grasps)

    def request_reset(self, level: str, timeout: float = 30.0) -> bool:
        self.events.clear()
        m = String(); m.data = level
        self.pub_reset.publish(m)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if any(e.get("type") == "reset_done" for e in self.events):
                self.events.clear()
                return True
            time.sleep(0.05)
        return False


def _xyz(msg):
    return [[p.position.x, p.position.y, p.position.z] for p in msg.poses]


def main() -> int:
    ap = argparse.ArgumentParser(description="task1 공구 전달 (code-as-policy)")
    ap.add_argument("--tool", required=True, choices=sorted(TOOL_NAMES),
                    help="전달할 공구")
    ap.add_argument("--yaw", type=float, default=0.0,
                    help="전달 요 각도 [deg] — 수직 아래 그리퍼를 이만큼 돌려 놓는다")
    ap.add_argument("--speed", type=float, default=0.2,
                    help="전달 속도 [m/s] (0.03~0.6)")
    ap.add_argument("--rate", type=float, default=6.0, help="제어 주기 [Hz]")
    ap.add_argument("--reset", choices=("none", "soft", "hard", "full"), default="full",
                    help="시작 전 초기화 강도 (기본 full)")
    args = ap.parse_args()
    prim = TOOL_NAMES[args.tool]

    rclpy.init()
    node = Bridge()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print("[deliver] 시뮬레이션 대기 중…", flush=True)
    t0 = time.time()
    while not node.ready() and time.time() - t0 < 30:
        time.sleep(0.1)
    if not node.ready():
        print("[deliver] 토픽이 오지 않습니다. task1 환경이 떠 있는지 확인하세요.", flush=True)
        return 1

    if args.reset != "none":
        print(f"[deliver] 초기화({args.reset}) 요청…", flush=True)
        node.request_reset(args.reset)
        time.sleep(2.0)

    policy = Task1DeliverPolicy(prim, yaw_deg=args.yaw, speed=args.speed)
    period = 1.0 / args.rate
    print(f"[deliver] {args.tool}({prim}) · 요 {policy.yaw_deg:g}° · "
          f"속도 {policy.speed:g} m/s", flush=True)

    done = False
    last = {}
    prev_stage = None
    ep_t0 = time.time()
    while not done and time.time() - ep_t0 < EPISODE_TIMEOUT_S:
        loop_t = time.time()
        if node.eef is None:
            continue
        st = node.status
        # "쥐고 있음" = 접촉력 OR 공구가 들려 있음. 얇은 공구(가위 1.6cm)는
        # 접촉력이 0 으로 몇 스텝씩 깜빡여서, 접촉만 보면 멀쩡히 들고 있는데
        # 오탐 ESCAPE 가 그리퍼를 열어 떨어뜨린다 (실측: z 0.088 유지 중 접촉 0).
        # 들려 있는 공구는 물리적으로 쥔 것 외에 설명이 없다.
        _tool = next((t for t in node.tools() if t["name"] == prim), None)
        _lifted = bool(_tool and _tool["pos"][2] > 0.055)
        gripping = (float((st.get("contact") or {}).get(prim, 0.0)) > 0.3) or _lifted
        delta, close, info = policy.act(node.eef, node.eef_quat, node.tools(),
                                        node.flange_offset(), gripping=gripping)
        node.send(delta, close)
        if info.get("stage") in ("LIFT", "DELIVER"):
            _t = next((t for t in node.tools() if t["name"] == prim), None)
            print(f"      dbg y={node.eef[1]:+.3f} 접촉={float((st.get('contact') or {}).get(prim, 0.0)):.2f} "
                  f"공구z={_t['pos'][2]:.3f}" if _t else "      dbg 공구없음", flush=True)
        if info.get("stage") != prev_stage:
            print(f"    [{info.get('stage')}] eef={[round(v,3) for v in node.eef]}",
                  flush=True)
            prev_stage = info.get("stage")
        if any(e.get("type") == "tool_crossed" and prim in (e.get("tools") or [])
               for e in node.events):
            node.send([0.0] * 6, False)
            print(f"[deliver] 성공 — {args.tool} 가 경계를 넘어 초기화됨", flush=True)
            return 0
        if info.get("abort"):
            print(f"[deliver] 실패 — {info.get('why')}", flush=True)
            node.send([0.0] * 6, False)
            return 2
        done = bool(info.get("done"))
        last = info
        sleep = period - (time.time() - loop_t)
        if sleep > 0:
            time.sleep(sleep)
    node.send([0.0] * 6, False)

    if not done:
        print(f"[deliver] 실패 — 시간 초과 (단계 {last.get('stage')})", flush=True)
        return 2

    print(f"[deliver] 실패 — 경계 통과 이벤트 없음 (단계 {last.get('stage')})",
          flush=True)
    return 2


if __name__ == "__main__":
    sys.exit(main())
