#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""task1 공구 전달 시연을 모아 LeRobot 형식으로 저장한다.

task3 수집기와 같은 뼈대(ROS 전용 연결, 별도 스핀 스레드, LeRobotWriter)를 쓰되
task1 에 맞게 세 가지가 다르다.

  1. 에피소드마다 (공구, 요각, 속도)를 뽑는다 — 공구는 덜 모인 쪽, 요각은
     0~360° 균등, 속도는 0.10~0.35 m/s 균등. 명령문은 공구별로 붙는다:
     "pass the hammer" / "pass the drill" (LeRobot task 필드, 에피소드별).
  2. 성공 판정은 시뮬레이션의 경계 통과 초기화 이벤트(tool_crossed)다 —
     공구가 노란 테이프를 실제로 넘어야 성공이고, 넘는 순간 시뮬레이션이
     공구를 초기화하므로 성공 에피소드 뒤에는 리셋이 필요 없다.
  3. 느린-이상치 필터가 **없다**. 전달 속도가 에피소드마다 다른 것이 의도라
     길이 편차는 데이터의 일부다. 시간 초과만 거른다.

실행:
    data_collection/task1/run.sh --per-tool 50    # _data/task1/<날짜시간> 에 저장
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
# LeRobotWriter 는 task3 것을 그대로 쓴다 — 저장 형식이 같아야 ingest 도 같이 쓴다.
# 뒤에 붙여야(append) 이름이 같은 policy 모듈이 task1 것으로 잡힌다.
sys.path.append(str(Path(__file__).resolve().parent.parent / "task3"))

import rclpy                                            # noqa: E402
from geometry_msgs.msg import PoseArray, PoseStamped, Twist   # noqa: E402
from rclpy.node import Node                             # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from sensor_msgs.msg import CompressedImage             # noqa: E402
from std_msgs.msg import Bool, Float32, String          # noqa: E402

from lerobot_writer import LeRobotWriter                # noqa: E402
from policy import Task1DeliverPolicy                   # noqa: E402

TOOLS = {"hammer": "hammer_7", "drill": "cordless_drill"}
TASK_TEXT = {"hammer": "pass the hammer", "drill": "pass the drill"}
CAMERAS = ("front", "wrist")
EPISODE_TIMEOUT_S = 150.0
YAW_RANGE = (0.0, 360.0)
SPEED_RANGE = (0.10, 0.35)
# 접촉력이 얇은 물체에서 0 으로 깜빡이므로 "물었다" 는 접촉 또는 들림으로 판정
# (deliver.py 와 동일 근거).
HOLD_Z = 0.055


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("task1_collector")
        self.eef = None
        self.eef_quat = None
        self.gripper = 0.0
        self.names: list[str] = []
        self.objects: list[list[float]] = []
        self.grasps: list[list[float]] = []
        self.ginfo: list[dict] = []
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
        self.create_subscription(String, "/franka/grasp_info",
                                 lambda m: setattr(self, "ginfo", json.loads(m.data)), 10)
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
        out = []
        n = min(len(self.names), len(self.objects), len(self.grasps))
        for i in range(n):
            out.append({"name": self.names[i], "pos": self.objects[i],
                        "flange": self.grasps[i]})
        return out

    def flange_offset(self) -> float:
        for i in range(min(len(self.objects), len(self.grasps))):
            return self.grasps[i][2] - self.objects[i][2]
        return 0.15

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
                and self.grasps and all(c in self.images for c in CAMERAS))

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
    ap = argparse.ArgumentParser(description="task1 공구 전달 시연 수집")
    ap.add_argument("--per-tool", type=int, default=50,
                    help="공구당 저장할 성공 에피소드 수")
    ap.add_argument("--out", type=str, default=None,
                    help="저장 위치. 생략하면 _data/task1/<날짜시간>")
    ap.add_argument("--seed", type=int, default=0, help="요각·속도 난수 시드")
    ap.add_argument("--rate", type=float, default=6.0, help="제어 주기 [Hz]")
    ap.add_argument("--max-attempts", type=int, default=0,
                    help="이만큼 시도하면 무조건 끝낸다 (0=무제한)")
    ap.add_argument("--reset", choices=("none", "soft", "hard", "full"), default="full")
    args = ap.parse_args()
    if args.out is None:
        from datetime import datetime
        args.out = ("/workspace/franka_robolab_sim/_data/task1/"
                    + datetime.now().strftime("%Y%m%d_%H%M%S"))

    rclpy.init()
    node = Collector()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print("[collect] 시뮬레이션 대기 중…", flush=True)
    t0 = time.time()
    while not node.ready() and time.time() - t0 < 30:
        time.sleep(0.1)
    if not node.ready():
        print("[collect] 토픽이 오지 않습니다. scripts/task1.sh 가 떠 있는지 확인하세요.",
              flush=True)
        return 1

    if args.reset != "none":
        print(f"[collect] 시뮬레이션 초기화 요청 ({args.reset})…", flush=True)
        if node.request_reset(args.reset):
            time.sleep(2.0)
        else:
            print("[collect] 경고: 초기화 응답 없음 — 현재 상태로 진행", flush=True)

    writer = LeRobotWriter(args.out, fps=args.rate, cameras=list(CAMERAS),
                           state_dim=8, action_dim=7, task=TASK_TEXT["hammer"])
    rng = random.Random(args.seed)
    period = 1.0 / args.rate
    saved = {"hammer": 0, "drill": 0}
    attempts = 0
    consec_fail = 0

    while sum(saved.values()) < 2 * args.per_tool:
        if args.max_attempts and attempts >= args.max_attempts:
            print(f"[collect] 시도 {attempts}회로 중단 (--max-attempts)", flush=True)
            break
        # 덜 모인 공구부터. 동률이면 시도 횟수로 번갈아 — 한쪽만 연달아 돌면
        # 실패 유형이 쏠려도 늦게 알아챈다.
        if saved["hammer"] == saved["drill"]:
            key = "hammer" if attempts % 2 == 0 else "drill"
        else:
            key = min(saved, key=saved.get)
        if saved[key] >= args.per_tool:
            key = "drill" if key == "hammer" else "hammer"
        yaw = rng.uniform(*YAW_RANGE)
        speed = rng.uniform(*SPEED_RANGE)
        prim = TOOLS[key]

        attempts += 1
        policy = Task1DeliverPolicy(prim, yaw_deg=yaw, speed=speed)
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
            tool_obj = next((t for t in node.tools() if t["name"] == prim), None)
            contact = float((st.get("contact") or {}).get(prim, 0.0))
            lifted = bool(tool_obj and tool_obj["pos"][2] > HOLD_Z)
            delta, close, info = policy.act(
                node.eef, node.eef_quat, node.tools(), node.flange_offset(),
                gripping=contact > 0.3 or lifted)
            node.send(delta, close)

            if info.get("stage") != "SEARCH":
                writer.add(
                    state=node.eef + node.eef_quat + [node.gripper],
                    action=[float(v) for v in delta] + [1.0 if close else 0.0],
                    images=dict(node.images),
                    extra={"stage": info.get("stage", ""), "target": prim,
                           "yaw": float(yaw), "speed": float(speed)},
                )
                frames += 1
            last = info
            aborted = bool(info.get("abort"))

            for e in list(node.events):
                if e.get("type") == "tool_crossed" and prim in (e.get("tools") or []):
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
            idx = writer.save_episode(task=TASK_TEXT[key])
            saved[key] += 1
            print(f"[collect] ep{idx} 저장 — {key} yaw={yaw:.0f}° v={speed:.2f} "
                  f"{frames}프레임 · 망치 {saved['hammer']}/{args.per_tool} "
                  f"드릴 {saved['drill']}/{args.per_tool} (시도 {attempts})", flush=True)
        else:
            consec_fail += 1
            writer.discard()
            why = last.get("why") or ("중단" if aborted else "시간 초과")
            print(f"[collect] 실패 — {key} yaw={yaw:.0f}° v={speed:.2f} · {why} "
                  f"단계={last.get('stage')} (시도 {attempts})", flush=True)
            # 실패하면 공구가 흐트러진 채 남는다 — 특히 드릴은 손잡이 방향이
            # 틀어지면 파지 오프셋(월드 상수)이 무효가 되므로 반드시 되돌린다.
            print("[collect] full 초기화로 복구", flush=True)
            node.request_reset("full")
            time.sleep(2.0)

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

    print(f"[collect] 완료: 망치 {saved['hammer']} + 드릴 {saved['drill']} = "
          f"{sum(saved.values())}개 저장 → {args.out} (시도 {attempts}회)", flush=True)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
