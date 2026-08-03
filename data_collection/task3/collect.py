#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""task3 학습 환경에서 pick-and-place 시연을 모아 LeRobot 형식으로 저장한다.

시뮬레이션과는 **ROS 로만** 이어진다. 시뮬레이터를 직접 import 하지 않으므로 같은
코드가 실물 로봇에도 붙는다.

    구독  /franka/eef_pose  /franka/gripper_state  /franka/objects
          /franka/object_names  /franka/grasp_poses  /franka/grasp_info
          /franka/status  /franka/camera/{front,wrist}/image_raw/compressed
    발행  /franka/cmd/eef_delta  /franka/cmd/gripper

실행:
    data_collection/task3/run.sh --episodes 20 --out ~/datasets/task3_train

에피소드는 "집어서 통에 담기" 한 번이다. 도중에 목표 캔이 회수되거나 시간이 초과되면
그 에피소드는 **버린다** — 실패 시연이 섞이면 학습이 흐려진다. 성공했더라도 다른
에피소드 중앙값보다 1.4배 넘게 길면 버린다(OUTLIER_FACTOR) — 유난히 느린 시연은
머뭇거림째로 학습되므로 짧고 고른 것만 남긴다.
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
from sensor_msgs.msg import CompressedImage             # noqa: E402
from std_msgs.msg import Bool, Float32, String          # noqa: E402

from lerobot_writer import LeRobotWriter                # noqa: E402
from policy import PickPlacePolicy                      # noqa: E402

TASK_TEXT = "Pick up the cans from the conveyor and put them in the bin"
CAMERAS = ("front", "wrist")
EPISODE_TIMEOUT_S = 150.0

# 다른 에피소드보다 이 배율 넘게 길면 "너무 오래 걸린" 것으로 보고 버린다.
# 성공했더라도 유난히 느린 시연은 머뭇거림째로 학습된다 — 짧고 고른 시연만 남긴다.
# 중앙값 기준이라 느린 것이 몇 개 섞여도 기준 자체가 끌려가지 않는다.
OUTLIER_FACTOR = 1.4
# 중앙값이 의미를 가지려면 이만큼은 쌓여야 한다. 그 전에는 일단 저장하고,
# 수집이 끝난 뒤 최종 중앙값으로 다시 걸러 낸다 (첫 에피소드가 유난히 길면
# 이 마지막 검사에서 걸린다).
OUTLIER_MIN_BASELINE = 4


def _outliers(episodes: list[dict]) -> list[int]:
    """느린 이상치 에피소드 번호들. episodes 는 writer 의 메타 그대로다."""
    import statistics
    if len(episodes) < OUTLIER_MIN_BASELINE:
        return []
    med = statistics.median(e["length"] for e in episodes)
    return [e["episode_index"] for e in episodes if e["length"] > OUTLIER_FACTOR * med]


class Collector(Node):
    def __init__(self, args) -> None:
        super().__init__("task3_collector")
        self.args = args
        self.eef = None
        self.eef_quat = None
        self.gripper = 0.0
        self.names: list[str] = []
        self.objects: list[list[float]] = []
        self.grasps: list[list[float]] = []
        self.ginfo: list[dict] = []
        self.status: dict = {}
        self.images: dict[str, bytes] = {}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, "/franka/eef_pose", self._on_eef, 10)
        self.create_subscription(Float32, "/franka/gripper_state",
                                 lambda m: setattr(self, "gripper", float(m.data)), 10)
        self.create_subscription(PoseArray, "/franka/objects",
                                 lambda m: setattr(self, "objects", _xyz(m)), 10)
        self.create_subscription(PoseArray, "/franka/grasp_poses",
                                 lambda m: setattr(self, "grasps", _xyz(m)), 10)
        self.belt_order: list[str] = []
        self.events: list[dict] = []
        self.create_subscription(String, "/franka/events",
                                 lambda m: self.events.append(json.loads(m.data)), 20)
        self.create_subscription(String, "/franka/belt_order",
                                 lambda m: setattr(self, "belt_order", json.loads(m.data)), 10)
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

    # ── 편의 ──────────────────────────────────────────────────────────
    def cans(self) -> list[dict]:
        """이름·자세·파지자세를 하나로 묶는다. 세 토픽의 순서는 같다."""
        out = []
        n = min(len(self.names), len(self.objects), len(self.grasps))
        for i in range(n):
            info = next((g for g in self.ginfo if g.get("name") == self.names[i]), {})
            out.append({
                "name": self.names[i],
                "pos": self.objects[i],
                "flange": self.grasps[i],
                "on_belt": bool(info.get("on_belt", False)),
                "order": int(info.get("order", -1)),
                "half_height": float(info.get("half_height", 0.03)),
            })
        return out

    def flange_offset(self) -> float:
        """손끝→플랜지 수직 거리. 파지 자세와 물체 자세의 차이가 곧 그 값이다."""
        for i in range(min(len(self.objects), len(self.grasps))):
            return self.grasps[i][2] - self.objects[i][2]
        return 0.15

    def send(self, delta, close) -> None:
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = delta[0], delta[1], delta[2]
        t.angular.x, t.angular.y, t.angular.z = delta[3], delta[4], delta[5]
        self.pub_delta.publish(t)
        b = Bool(); b.data = bool(close)
        self.pub_grip.publish(b)

    def ready(self) -> bool:
        return (self.eef is not None and self.names and self.objects
                and self.grasps and all(c in self.images for c in CAMERAS))

    def request_reset(self, level: str, timeout: float = 30.0) -> bool:
        """시뮬레이션 초기화를 요청하고 완료(reset_done)를 기다린다.

        수집을 시작할 때마다 프로세스를 죽였다 살리는 대신 이걸 쓴다. 씬을 다시
        로드하는 데 2분이 걸리지만 이 경로는 한 스텝이면 끝나고, full 이면 화물
        자세와 벨트 장부(회수·투입 수)까지 기동 직후로 돌아온다 — 실행마다 같은
        조건에서 시작해야 성공률 비교가 의미를 갖는다.
        """
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
    ap = argparse.ArgumentParser(description="task3 시연 수집 (code-as-policy)")
    ap.add_argument("--episodes", type=int, default=10,
                    help="최종 데이터셋에 남길 에피소드 수. 실패와 느린 이상치를 "
                         "거른 뒤의 개수라, 시도는 이보다 많아질 수 있다.")
    ap.add_argument("--out", type=str, required=True, help="데이터셋 저장 위치")
    ap.add_argument("--seed", type=int, default=0, help="대상 선택 난수 시드")
    ap.add_argument("--rate", type=float, default=6.0, help="제어 주기 [Hz]")
    ap.add_argument("--max-attempts", type=int, default=0,
                    help="이만큼 시도하면 성공 여부와 무관하게 끝낸다 (0=무제한). "
                         "튜닝할 때 한 번 돌리는 시간을 잘라 준다.")
    ap.add_argument("--reset", choices=("none", "soft", "hard", "full"), default="full",
                    help="시작 전에 시뮬레이션을 초기화한다 (기본 full). 시뮬레이터를 "
                         "죽였다 살릴 필요 없이 매번 같은 조건에서 시작하기 위한 것이다. "
                         "직전 상태를 이어서 보고 싶으면 none.")
    args = ap.parse_args()

    rclpy.init()
    node = Collector(args)

    # 스핀은 **별도 스레드**에서 돌린다. 제어 루프 안에서 spin_once 를 부르면 한 번에
    # 콜백 하나만 처리되는데, 구독이 9개라 대부분의 토픽이 굶어 상태가 서로 어긋난다
    # (실제로 파지 정보만 낡아서 "목표 캔이 사라짐" 으로 매번 중단됐다).
    # 콜백은 속성 대입만 하므로 이 정도 경쟁은 문제되지 않는다.
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print("[collect] 시뮬레이션 대기 중…", flush=True)
    t0 = time.time()
    while not node.ready() and time.time() - t0 < 30:
        time.sleep(0.1)
    if not node.ready():
        print("[collect] 토픽이 오지 않습니다. 시뮬레이션이 떠 있는지 확인하세요.", flush=True)
        return 1

    if args.reset != "none":
        print(f"[collect] 시뮬레이션 초기화 요청 ({args.reset})…", flush=True)
        if node.request_reset(args.reset):
            # 초기화 직후에는 벨트가 비어 있다. 화물이 입구에 다시 놓이고 판정에
            # 쓰는 토픽(grasp_info 의 on_belt/order)이 갱신될 시간을 준다.
            time.sleep(2.0)
            # /status 에는 벨트 위 개수가 없다 (runner 가 몇 개만 골라 싣는다).
            # 같은 것을 알려 주는 /belt_order 를 쓴다.
            print(f"[collect] 초기화 완료 — 벨트 위 {len(node.belt_order)}개", flush=True)
        else:
            print("[collect] 경고: 초기화 응답이 없어 현재 상태 그대로 진행합니다.",
                  flush=True)

    writer = LeRobotWriter(args.out, fps=args.rate, cameras=list(CAMERAS),
                           state_dim=8, action_dim=7, task=TASK_TEXT)
    policy = PickPlacePolicy(seed=args.seed)
    period = 1.0 / args.rate
    saved, attempts, choices = 0, 0, {0: 0, 1: 0}

    while True:
        if saved >= args.episodes:
            # 초반(중앙값이 서기 전)에 저장된 느린 에피소드를 최종 기준으로 걷어낸다.
            bad = _outliers(writer.episodes)
            if not bad:
                break
            writer.prune_episodes(bad)
            saved = len(writer.episodes)
            print(f"[collect] 느린 에피소드 {len(bad)}개 제거 — "
                  f"{saved}/{args.episodes}, 더 모읍니다", flush=True)
            continue
        if args.max_attempts and attempts >= args.max_attempts:
            print(f"[collect] 시도 {attempts}회로 중단 (--max-attempts)", flush=True)
            break
        attempts += 1
        policy.reset()
        writer.discard()
        ep_t0 = time.time()
        last = {}
        _prev_stage = None
        binned0 = node.status.get("binned", 0)
        done = aborted = False
        stages = []

        while not done and not aborted and time.time() - ep_t0 < EPISODE_TIMEOUT_S:
            loop_t = time.time()
            if node.eef is None:
                continue

            st = node.status
            # 그리퍼가 작업 구역에 들어가면 벨트가 멈추므로, 그때는 예측 보정과
            # 속도 피드포워드를 모두 꺼야 한다 — 안 그러면 멈춘 캔을 앞질러 간다.
            belt_mps = (0.0 if st.get("belt_held")
                        else float(st.get("belt_mpm") or 0.0) / 60.0)
            # /status 의 contact 는 객체별 필터 접촉력이다 (runner 가
            # force_matrix_w 를 읽는다). 목표 캔의 키만 보면 "지금 그 캔을
            # 물었는가" 가 된다 — table 등 다른 키가 섞여 있어도 무관하다.
            tgt_name = policy.target_name
            gripping = float((st.get("contact") or {}).get(tgt_name, 0.0)) > 0.3
            delta, close, info = policy.act(
                node.eef, node.eef_quat, node.cans(), node.flange_offset(),
                belt_mps=belt_mps, gripping=gripping,
                belt_order=list(node.belt_order))
            node.send(delta, close)

            if info.get("stage") != "SEARCH":
                writer.add(
                    state=node.eef + node.eef_quat + [node.gripper],
                    action=list(delta) + [1.0 if close else 0.0],
                    images=dict(node.images),
                    extra={"stage": info.get("stage", ""),
                           "target": info.get("target") or ""},
                )
                stages.append(info.get("stage"))
            if "choice" in info:
                choices[info["choice"]] = choices.get(info["choice"], 0) + 1
            last = info
            # 처음 두 시도만 단계 전이를 찍는다 — 문제가 생기면 로그만으로
            # 어느 단계에서 멈췄는지 보이도록. 그 뒤로는 에피소드 요약만 남긴다.
            if attempts <= 2:
                stg = info.get("stage")
                if stg != _prev_stage:
                    tgt = next((c for c in node.cans()
                                if c["name"] == info.get("target")), None)
                    at = (f"Δx={node.eef[0]-tgt['pos'][0]:+.3f} "
                          f"Δy={node.eef[1]-tgt['pos'][1]:+.3f}" if tgt else "-")
                    print(f"    [{stg}] eef_z={node.eef[2]:.3f} {at} "
                          f"err={info.get('err','-')}", flush=True)
                _prev_stage = stg
            if any(e.get("type") == "gripper_explosion" for e in node.events):
                node.events.clear()
                aborted = True
                last = {**info, "why": "그리퍼 폭주 — 시뮬레이션이 리셋됨"}
                break
            if node.status.get("exploded"):
                aborted = True
                last = {**info, "why": "그리퍼 폭주(링키지 분해)"}
                break
            done = bool(info.get("done"))
            aborted = bool(info.get("abort"))

            sleep = period - (time.time() - loop_t)
            if sleep > 0:
                time.sleep(sleep)

        # 상태 기계가 끝까지 돌았다고 성공이 아니다. 캔이 **실제로 통에 들어갔는지**
        # 시뮬레이션의 binned 카운터로 확인한다. 이걸 안 보면 캔을 놓친 시연이
        # 성공으로 저장된다(실제로 225프레임짜리 실패가 저장됐다).
        if done:
            for _ in range(12):                 # 낙하가 감지될 시간
                time.sleep(period)
            if node.status.get("binned", 0) <= binned0:
                done = False
                print("[collect] 실패 — 통에 들어가지 않음 (놓쳤거나 빗나감)", flush=True)

        if done and len(writer.episodes) >= OUTLIER_MIN_BASELINE:
            import statistics
            med = statistics.median(e["length"] for e in writer.episodes)
            if len(stages) > OUTLIER_FACTOR * med:
                writer.discard()
                done = False
                print(f"[collect] 버림 — 너무 느림 ({len(stages)}프레임 > "
                      f"중앙값 {med:.0f}×{OUTLIER_FACTOR}) (시도 {attempts})", flush=True)

        if done:
            idx = writer.save_episode()
            saved += 1
            print(f"[collect] 에피소드 {idx} 저장 — {len(stages)}프레임, "
                  f"시도 {attempts}회, 1번째/2번째 = {choices.get(0,0)}/{choices.get(1,0)}",
                  flush=True)
        else:
            st = node.status
            where = (f"단계={last.get('stage')} 목표={last.get('target')} "
                     f"err={last.get('err')} eef={[round(v, 3) for v in node.eef]} "
                     f"finger_q={st.get('finger_q')}")
            if aborted:
                tgt = next((c for c in node.cans()
                            if c["name"] == last.get("target")), None)
                reason = (f"{last.get('why') or '목표가 벨트에서 빠짐'} · "
                          f"목표={last.get('target')} "
                          f"pos={[round(v,3) for v in tgt['pos']] if tgt else None} "
                          f"order={node.belt_order} "
                          f"off_belt={node.status.get('off_belt')}")
            elif last.get("done"):
                reason = "통 판정 실패"
            else:
                reason = f"시간 초과 · {where}"
            writer.discard()
            print(f"[collect] 실패 — {reason} (시도 {attempts})", flush=True)
        node.send([0.0] * 6, False)

    print(f"[collect] 완료: {saved}개 저장 → {args.out}", flush=True)
    print(f"[collect] 대상 선택 분포 — 첫 번째 {choices.get(0,0)}회 · "
          f"두 번째 {choices.get(1,0)}회", flush=True)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
