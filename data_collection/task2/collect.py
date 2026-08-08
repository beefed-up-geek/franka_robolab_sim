#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""task2 충전 연결 시연을 모아 LeRobot 형식으로 저장한다.

task1·task3 수집기와 같은 뼈대(ROS 전용 연결, 별도 스핀 스레드,
LeRobotWriter)를 쓰되 task2 에 맞게 다른 점:

  1. 한 에피소드 = 커넥터 하나를 집어 단자에 씌우기. 빨강(B+) 먼저 꽂고
     검정(A-) 을 꽂는 순서가 실무 규칙이라, 빨강 에피소드가 성공해야
     검정 에피소드를 시작한다. 검정까지 끝나면 시뮬레이션이
     charging_done 과 함께 커넥터를 초기화하므로 다음 쌍으로 이어진다.
  2. 성공 판정은 시뮬레이션의 connector_attached 이벤트다.
  3. 궤적·속도·정지 이벤트는 정책이 시드로 뽑는다 (policy.py v2) —
     에피소드마다 다른 z 물결·속도·일시 정지가 데이터에 들어간다.

실행:
    data_collection/task2/collect.sh --episodes 200
"""
from __future__ import annotations

import argparse
import json
import random
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.append(str(Path(__file__).resolve().parent.parent / "task3"))

import rclpy                                            # noqa: E402
from geometry_msgs.msg import PoseArray, PoseStamped, Twist   # noqa: E402
from rclpy.node import Node                             # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from sensor_msgs.msg import CompressedImage             # noqa: E402
from std_msgs.msg import Bool, Float32, String          # noqa: E402

from lerobot_writer import LeRobotWriter                # noqa: E402
from policy import Task2ConnectPolicy                   # noqa: E402

CAMERAS = ("front", "top", "wrist")
TASK_TEXT = {
    "connector_red": "Plug the red charging connector into the battery positive terminal",
    "connector_black": "Plug the black charging connector into the battery negative terminal",
}
SEQ = ("connector_red", "connector_black")
TERMS_LOCAL = {"connector_red": (0.0962, -0.0546, 0.208),
               "connector_black": (-0.0962, -0.0523, 0.208)}
EPISODE_TIMEOUT_S = 150.0
HOLD_Z = 0.05


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("task2_collector")
        self.eef = None
        self.eef_quat = None
        self.gripper = 0.0
        self.names: list[str] = []
        self.objects: list[list[float]] = []
        self.grasps: list[list[float]] = []
        self.status: dict = {}
        self.images: dict[str, bytes] = {}
        self.events: list[dict] = []

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, "/franka/eef_pose", self._on_eef, 10)
        self.create_subscription(Float32, "/franka/gripper_state",
                                 lambda m: setattr(self, "gripper", float(m.data)), 10)
        self.create_subscription(PoseArray, "/franka/objects",
                                 lambda m: setattr(self, "objects", _xyz(m)), 10)
        self.create_subscription(PoseArray, "/franka/grasp_poses",
                                 lambda m: setattr(self, "grasps", _xyz(m)), 10)
        self.create_subscription(String, "/franka/events",
                                 lambda m: self.events.append(json.loads(m.data)), 20)
        self.create_subscription(String, "/franka/status",
                                 lambda m: setattr(self, "status", json.loads(m.data)), 10)
        self.create_subscription(String, "/franka/object_names",
                                 lambda m: setattr(self, "names", json.loads(m.data)), latched)
        for cam in CAMERAS:
            self.create_subscription(
                CompressedImage, f"/franka/camera/{cam}/image_raw/compressed",
                lambda m, c=cam: self.images.__setitem__(c, bytes(m.data)), 2)

        self.pub_delta = self.create_publisher(Twist, "/franka/cmd/eef_delta", 10)
        self.pub_grip = self.create_publisher(Bool, "/franka/cmd/gripper", 10)
        self.pub_reset = self.create_publisher(String, "/franka/cmd/reset", 10)

    def _on_eef(self, m) -> None:
        p, o = m.pose.position, m.pose.orientation
        self.eef = [p.x, p.y, p.z]
        self.eef_quat = [o.w, o.x, o.y, o.z]

    def tools(self) -> list[dict]:
        return [{"name": n, "pos": p} for n, p in zip(self.names, self.objects)]

    def flange_offset(self) -> float:
        return 0.145

    def terminal(self, name: str):
        key = "pos" if name == "connector_red" else "neg"
        pub = (self.status.get("terminals") or {}).get(key)
        if pub:
            return list(pub)
        bat = next((t for t in self.tools() if t["name"] == "battery"), None)
        if bat is None:
            return None
        lx, ly, lz = TERMS_LOCAL[name]
        return [bat["pos"][0] - lx, bat["pos"][1] - ly, bat["pos"][2] + lz]

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
        return (self.eef is not None and self.names and self.objects
                and all(c in self.images for c in CAMERAS))

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


def _xyz(msg) -> list[list[float]]:
    return [[p.position.x, p.position.y, p.position.z] for p in msg.poses]


def main() -> int:
    ap = argparse.ArgumentParser(description="task2 충전 연결 시연 수집")
    ap.add_argument("--episodes", type=int, default=200,
                    help="저장할 성공 에피소드 총 수 (빨강·검정 합)")
    ap.add_argument("--out", type=str, default=None,
                    help="저장 위치. 생략하면 _data/task2/<날짜시간>")
    ap.add_argument("--seed", type=int, default=0, help="궤적·속도 난수 시드")
    ap.add_argument("--rate", type=float, default=6.0, help="제어 주기 [Hz]")
    ap.add_argument("--max-attempts", type=int, default=0,
                    help="이만큼 시도하면 무조건 끝낸다 (0=무제한)")
    ap.add_argument("--reset", choices=("none", "soft", "hard", "full"), default="full")
    args = ap.parse_args()
    if args.out is None:
        from datetime import datetime
        args.out = ("/workspace/franka_robolab_sim/_data/task2/"
                    + datetime.now().strftime("%Y%m%d_%H%M%S"))

    rclpy.init()
    node = Collector()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print("[collect] 시뮬레이션 대기 중…", flush=True)
    t0 = time.time()
    while not node.ready() and time.time() - t0 < 90:
        time.sleep(0.1)
    if not node.ready():
        print("[collect] 토픽이 오지 않습니다. sim_start.sh task2_train 확인.", flush=True)
        return 1

    if args.reset != "none":
        print(f"[collect] 시뮬레이션 초기화 요청 ({args.reset})…", flush=True)
        node.request_reset(args.reset)
        time.sleep(2.0)

    writer = LeRobotWriter(args.out, fps=args.rate, cameras=list(CAMERAS),
                           state_dim=4, action_dim=4,
                           task=TASK_TEXT["connector_red"])
    rng = random.Random(args.seed)
    period = 1.0 / args.rate
    saved = 0
    attempts = 0
    consec_fail = 0
    slot = 0            # 0=빨강, 1=검정 — 실무 순서를 지킨다

    while saved < args.episodes:
        if args.max_attempts and attempts >= args.max_attempts:
            print(f"[collect] 시도 {attempts}회로 중단 (--max-attempts)", flush=True)
            break
        name = SEQ[slot]
        term = node.terminal(name)
        if term is None:
            print("[collect] 배터리 관측 없음 — full 초기화", flush=True)
            node.request_reset("full")
            time.sleep(2.0)
            continue

        attempts += 1
        ep_rng = random.Random(args.seed * 100003 + attempts)
        policy = Task2ConnectPolicy(name, term, rng=ep_rng)
        writer.discard()
        node.events.clear()
        ep_t0 = time.time()
        frames = 0
        done = aborted = exploded = False
        last = {}

        while not done and not aborted and time.time() - ep_t0 < EPISODE_TIMEOUT_S:
            loop_t = time.time()
            if node.eef is None:
                continue
            st = node.status
            tool = next((t for t in node.tools() if t["name"] == name), None)
            contact = float((st.get("contact") or {}).get(name, 0.0))
            lifted = bool(tool and tool["pos"][2] > HOLD_Z)
            delta, close, info = policy.act(
                node.eef, node.eef_quat, node.tools(), node.flange_offset(),
                gripping=contact > 0.12 or lifted)
            node.send(delta, close)

            if info.get("stage") != "SEARCH":
                writer.add(
                    state=node.eef + [node.gripper],
                    action=[float(delta[0]), float(delta[1]), float(delta[2]),
                            1.0 if close else 0.0],
                    images=dict(node.images),
                    extra={"stage": info.get("stage", ""), "target": name,
                           "traj": int(policy.family), "speed": float(policy.speed)},
                )
                frames += 1
            last = info
            aborted = bool(info.get("abort"))

            for e in list(node.events):
                if e.get("type") == "connector_attached" and e.get("connector") == name:
                    done = True
                if e.get("type") == "gripper_explosion":
                    aborted = exploded = True
                    last = {**info, "why": "그리퍼 폭주"}
            if node.status.get("exploded"):
                aborted = exploded = True
                last = {**info, "why": "그리퍼 폭주(링키지 분해)"}
            node.events.clear()

            sleep = period - (time.time() - loop_t)
            if sleep > 0:
                time.sleep(sleep)

        node.send([0.0] * 6, False)

        if done:
            consec_fail = 0
            # 손을 놓고 위로 물러난다 — 다음 에피소드의 시작 자세가 된다
            for _ in range(8):
                node.send([0.0, 0.0, 0.06, 0.0, 0.0, 0.0], False)
                time.sleep(period)
            node.send([0.0] * 6, False)
            idx = writer.save_episode(task=TASK_TEXT[name])
            saved += 1
            print(f"[collect] ep{idx} 저장 — {name} {policy.describe()} "
                  f"{frames}프레임 · {saved}/{args.episodes} (시도 {attempts})",
                  flush=True)
            slot = 1 - slot
            if slot == 0:
                # 검정까지 끝났다 — 시뮬레이션의 완료 초기화를 기다린다
                t1 = time.time()
                while time.time() - t1 < 15:
                    if any(e.get("type") == "charging_done" for e in node.events):
                        break
                    time.sleep(0.2)
                node.events.clear()
                time.sleep(1.0)
        else:
            consec_fail += 1
            writer.discard()
            why = last.get("why") or ("중단" if aborted else "시간 초과")
            print(f"[collect] 실패 — {name} {policy.describe()} · {why} "
                  f"단계={last.get('stage')} (시도 {attempts})", flush=True)
            node.request_reset("full")
            time.sleep(2.0)
            slot = 0        # 초기화되면 빨강부터 다시

        if exploded or consec_fail >= 3:
            why = "폭주 후" if exploded else f"{consec_fail}연속 실패"
            consec_fail = 0
            print(f"[collect] {why} 회복 — full 초기화 + 안정 대기", flush=True)
            node.request_reset("full")
            t1 = time.time()
            while time.time() - t1 < 30:
                if not node.status.get("exploded"):
                    break
                time.sleep(0.5)
            time.sleep(2.0)
            slot = 0

    print(f"[collect] 완료: {saved}개 저장 → {args.out} (시도 {attempts}회)", flush=True)
    # spin 데몬 스레드가 살아있는 채 인터프리터가 내려가면 DDS 소멸자
    # 경합으로 코어 덤프가 난다 — 컨텍스트를 닫고 스레드를 거둔다.
    rclpy.try_shutdown()
    spinner.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
