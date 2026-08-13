# SPDX-License-Identifier: Apache-2.0
"""시뮬레이션을 ROS 2 노드로 노출한다.

Isaac Sim 이 ROS 2 를 통째로 번들하고 있어서 **시뮬레이션 프로세스 안에서** 노드를
띄운다. 시스템 ROS 설치는 필요 없다. 다만 번들 rclpy 는 기본 경로에 없어서
PYTHONPATH·LD_LIBRARY_PATH 를 잡아 줘야 import 된다 — scripts/sim_start.sh 가
`ROS_BUNDLE` 로 넣어 준다.

    /franka/eef_pose            geometry_msgs/PoseStamped   EEF 위치·자세
    /franka/gripper_state       std_msgs/Float32            0=열림 1=닫힘
    /franka/objects             geometry_msgs/PoseArray     화물 자세 (순서 고정)
    /franka/object_names        std_msgs/String             화물 이름 JSON 배열
    /franka/status              std_msgs/String             제어율·벨트·대기열 JSON
    /franka/events              std_msgs/String             이상 상황 알림 JSON
                                (그리퍼 폭주 등 — 구독자가 그 구간 데이터를 버릴 수 있게)
    /franka/camera/front/image_raw/compressed   sensor_msgs/CompressedImage
    /franka/camera/wrist/image_raw/compressed   sensor_msgs/CompressedImage
    /franka/camera/top/image_raw/compressed     sensor_msgs/CompressedImage

    /franka/cmd/eef_delta       geometry_msgs/Twist         EEF 델타 (구독)
    /franka/cmd/gripper         std_msgs/Bool               True=닫기 (구독)
    /franka/cmd/reset           std_msgs/String             초기화 (구독)
                                soft(기본) / hard / full — franka_env/config.py 참고.
                                full 이면 프로세스를 다시 띄운 것과 같은 상태가 되므로
                                수집기가 심을 죽였다 살릴 필요가 없다.

스핀은 심 스레드가 매 스텝 `spin_once(timeout_sec=0)` 로 돌린다. 별도 스레드를 두면
TeleopState 의 락과 얽혀 입력 순서가 꼬인다.

rclpy 를 못 찾으면 조용히 꺼진 채로 동작한다 — ROS 없이도 브라우저 조작은 되어야
하고, 그것 때문에 시뮬레이션이 뜨지 않으면 안 된다.
"""
from __future__ import annotations

import json


class RosBridge:
    """ROS 2 발행·구독. rclpy 가 없으면 모든 메서드가 무동작이다."""

    def __init__(self, state, namespace: str = "/franka", enabled: bool = True) -> None:
        self.state = state
        self.ok = False
        self._node = None
        self._objects_len = 0
        if not enabled:
            print("[ros] 비활성 (--no-ros)", flush=True)
            return
        try:
            import rclpy
            from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
            from sensor_msgs.msg import CompressedImage
            from rclpy.qos import DurabilityPolicy, QoSProfile
            from std_msgs.msg import Bool, Float32, String
        except Exception as exc:                      # noqa: BLE001
            print(f"[ros] rclpy 를 불러오지 못해 ROS 를 끕니다: {exc}", flush=True)
            return

        self._rclpy = rclpy
        self._Pose = Pose
        self._PoseStamped, self._PoseArray = PoseStamped, PoseArray
        self._CompressedImage = CompressedImage
        self._String, self._Float32 = String, Float32

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("franka_sim")
        ns = namespace.rstrip("/")

        q = 10
        self.pub_pose = self._node.create_publisher(PoseStamped, f"{ns}/eef_pose", q)
        self.pub_grip = self._node.create_publisher(Float32, f"{ns}/gripper_state", q)
        self.pub_objs = self._node.create_publisher(PoseArray, f"{ns}/objects", q)
        # 이름 목록은 값이 거의 안 바뀌므로 **latched** 로 낸다. 그냥 보내면
        # 나중에 붙은 구독자가 영영 못 받는다 (실제로 한 번 놓쳤다).
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_names = self._node.create_publisher(String, f"{ns}/object_names", latched)
        self.pub_status = self._node.create_publisher(String, f"{ns}/status", q)
        # 파지 자세 — /objects 와 **순서와 길이가 같다.** 붙여서 쓰라고 따로 낸다.
        self.pub_grasp = self._node.create_publisher(PoseArray, f"{ns}/grasp_poses", q)
        self.pub_ginfo = self._node.create_publisher(String, f"{ns}/grasp_info", q)
        # 벨트에 **지금 올라와 있는** 화물만, 출구에 가까운 순서로. 정책이 이것만
        # 보면 되도록 따로 낸다 — objects/grasp_poses 는 대기열 화물까지 포함하고
        # 순서가 고정이라, 거기서 고르면 상판 아래 화물을 집으러 가게 된다.
        self.pub_order = self._node.create_publisher(String, f"{ns}/belt_order", q)
        # 이벤트는 드물지만 놓치면 안 되므로 큐를 넉넉히 잡는다.
        self.pub_event = self._node.create_publisher(String, f"{ns}/events", 20)
        self.pub_img = {
            name: self._node.create_publisher(
                CompressedImage, f"{ns}/camera/{name}/image_raw/compressed", 2
            )
            for name in ("front", "top", "wrist", "view")
        }

        self._node.create_subscription(Twist, f"{ns}/cmd/eef_delta", self._on_delta, q)
        self._node.create_subscription(Bool, f"{ns}/cmd/gripper", self._on_gripper, q)
        self._node.create_subscription(String, f"{ns}/cmd/reset", self._on_reset, q)
        self._node.create_subscription(Float32, f"{ns}/cmd/belt", self._on_belt, q)

        self.ok = True
        print(f"[ros] 노드 franka_sim 기동 — 네임스페이스 {ns}", flush=True)

    # ── 구독 ────────────────────────────────────────────────────────────
    def _on_delta(self, msg) -> None:
        """EEF 델타를 키보드 입력과 같은 자리에 넣는다.

        값은 **한 스텝 분량의 이동량[m]·회전량[rad]** 이다. 키보드가 만들어 내는
        델타와 같은 단위라야 안전 클램프가 그대로 걸린다.
        """
        self.state.set_external_delta([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        ])

    def _on_gripper(self, msg) -> None:
        self.state.set_external_gripper(bool(msg.data))

    def _on_belt(self, msg) -> None:
        """벨트 속도 설정 [m/분]. 수집기가 에피소드마다 속도를 바꿀 때 쓴다."""
        self.state.set_belt_mpm(float(msg.data))

    def _on_reset(self, msg) -> None:
        """초기화 요청. 빈 문자열이면 soft.

        여기서 리셋하지 않고 TeleopState 에 요청만 걸어 둔다 — 물리 상태를
        만지는 일은 전부 심 스레드 소유이고, 콜백은 그 스레드의 spin() 안에서
        돌지만 순서 보장이 없다. 심 루프가 다음 스텝 첫머리에 가져간다.
        """
        want = (msg.data or "").strip().lower()
        got = self.state.request_reset(want)
        print(f"[ros] 초기화 요청 {want or '(빈값)'} → {got}", flush=True)

    # ── 발행 ────────────────────────────────────────────────────────────
    def event(self, kind: str, **fields) -> None:
        """이상 상황을 알린다. 데이터 수집기가 그 구간을 버릴 수 있게 하는 용도다."""
        if not self.ok:
            return
        m = self._String()
        m.data = json.dumps({"type": kind, **fields}, ensure_ascii=False)
        self.pub_event.publish(m)

    def publish(self, *, eef_pos=None, eef_quat=None, gripper=None,
                objects=None, grasps=None, belt_order=None, images=None,
                status=None) -> None:
        if not self.ok:
            return
        node = self._node
        now = node.get_clock().now().to_msg()

        if eef_pos is not None:
            m = self._PoseStamped()
            m.header.stamp, m.header.frame_id = now, "world"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, eef_pos)
            if eef_quat is not None:
                # Isaac 은 (w,x,y,z), ROS 는 (x,y,z,w) 순서다. 바꿔 주지 않으면
                # 자세가 조용히 어긋난 채로 나간다.
                w, x, y, z = (float(v) for v in eef_quat)
                m.pose.orientation.x, m.pose.orientation.y = x, y
                m.pose.orientation.z, m.pose.orientation.w = z, w
            self.pub_pose.publish(m)

        if gripper is not None:
            g = self._Float32()
            g.data = float(gripper)
            self.pub_grip.publish(g)

        if objects:
            arr = self._PoseArray()
            arr.header.stamp, arr.header.frame_id = now, "world"
            names = []
            for name, (pos, quat) in objects.items():
                pose = self._Pose()
                pose.position.x, pose.position.y, pose.position.z = map(float, pos)
                w, x, y, z = (float(v) for v in quat)
                pose.orientation.x, pose.orientation.y = x, y
                pose.orientation.z, pose.orientation.w = z, w
                arr.poses.append(pose)
                names.append(name)
            self.pub_objs.publish(arr)
            # 이름은 자세와 순서가 같다. latched 라 한 번만 보내면 된다.
            if len(names) != self._objects_len:
                s = self._String()
                s.data = json.dumps(names, ensure_ascii=False)
                self.pub_names.publish(s)
                self._objects_len = len(names)

        if grasps:
            arr = self._PoseArray()
            arr.header.stamp, arr.header.frame_id = now, "world"
            info = []
            for name, g in grasps.items():
                pose = self._Pose()
                pose.position.x, pose.position.y, pose.position.z = map(float, g["flange"])
                w, x, y, z = (float(v) for v in g["quat"])
                pose.orientation.x, pose.orientation.y = x, y
                pose.orientation.z, pose.orientation.w = z, w
                arr.poses.append(pose)
                info.append({k: v for k, v in g.items() if k not in ("flange", "quat")}
                            | {"name": name})
            self.pub_grasp.publish(arr)
            s = self._String()
            s.data = json.dumps(info, ensure_ascii=False)
            self.pub_ginfo.publish(s)

        if belt_order is not None:
            s = self._String()
            s.data = json.dumps(belt_order, ensure_ascii=False)
            self.pub_order.publish(s)

        for name, jpeg in (images or {}).items():
            pub = self.pub_img.get(name)
            if pub is None or not jpeg:
                continue
            img = self._CompressedImage()
            img.header.stamp, img.header.frame_id = now, f"{name}_camera"
            img.format = "jpeg"
            img.data = jpeg
            pub.publish(img)

        if status is not None:
            s = self._String()
            s.data = json.dumps(status, ensure_ascii=False)
            self.pub_status.publish(s)

    def spin(self) -> None:
        """쌓인 구독 콜백을 처리한다. 블로킹하지 않는다.

        spin_once 는 한 번에 콜백 **하나**만 처리한다. 퍼블리셔가 Twist 와 Bool 을
        각각 제어 주기로 보내면 한 번으로는 처리량이 모자라 큐가 밀리고, 명령이
        한 박자 늦게 반영된다. 몇 번 돌려 비운다 — 처리할 게 없으면 즉시 돌아오므로
        비용은 거의 없다.
        """
        if self.ok:
            for _ in range(8):
                self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def shutdown(self) -> None:
        if not self.ok:
            return
        try:
            self._node.destroy_node()
            self._rclpy.shutdown()
        except Exception:                              # noqa: BLE001
            pass
        self.ok = False
