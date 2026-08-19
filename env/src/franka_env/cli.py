# SPDX-License-Identifier: Apache-2.0
"""환경 실행 스크립트가 공통으로 쓰는 인자 정의.

이 모듈은 Isaac Sim 을 띄우기 **전에** import 된다. 그래서 isaaclab·robolab·torch
같은 것을 절대 import 하면 안 된다 — argparse 만 쓴다.

환경마다 기본값이 다를 수 있으므로 `build_parser(task=..., camera=...)` 로 기본값을
바꿔 쓴다. 실제 환경 구성은 env/script 의 각 스크립트가 정한다.
"""
from __future__ import annotations

import argparse

# config.VIEW_PRESETS 와 **손으로 맞춰야 한다.** 이 모듈은 Isaac Sim 을 띄우기 전에
# import 되므로 franka_env.config 밖의 것을 불러올 수 없다.
VIEW_CHOICES = ("behind", "front")
CONVEYOR_MODES = ("script", "force", "surface", "none")


def build_parser(
    description: str,
    task: str,
    view: str = "behind",
    conveyor: str = "script",
) -> argparse.ArgumentParser:
    """환경 실행 스크립트용 인자 파서를 만든다.

    Args:
        description: `--help` 에 나올 설명.
        task: 기본 태스크 이름 (env/src/tasks 의 클래스 이름).
        view: 궤도 카메라의 시작 자세. front·wrist 화면은 언제나 함께 나온다.
        conveyor: 기본 컨베이어 구동 방식.
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--task", type=str, default=task, help="env/src/tasks 의 태스크 클래스 이름")
    parser.add_argument(
        "--view",
        type=str,
        default=view,
        choices=VIEW_CHOICES,
        help="궤도 카메라의 시작 자세. behind 가 키 방향과 화면 방향이 일치한다. "
             "정면·손목 화면은 인자와 무관하게 항상 함께 송출된다.",
    )
    parser.add_argument(
        "--stream-width", type=int, default=960, help="브라우저로 보낼 영상 가로 폭 [px]"
    )
    parser.add_argument(
        "--physics-device",
        type=str,
        default="cpu",
        help="물리 연산 디바이스. 환경이 1개뿐이라 CPU 가 GPU 보다 빠르다 (11Hz vs 4.5Hz).",
    )
    parser.add_argument(
        "--conveyor",
        type=str,
        default=conveyor,
        choices=CONVEYOR_MODES,
        help="script=위치를 직접 전진(기본, 이 환경에서 유일하게 동작). "
             "none=벨트 없는 태스크(task1) — 컨베이어 로직 전체 비활성. "
             "force=외력으로 마찰 흉내 — 외력이 반영되지 않아 동작하지 않는다. "
             "surface=PhysX 표면 속도 — 콜라이더가 깨져 동작하지 않는다.",
    )
    parser.add_argument(
        "--belt-speed", type=float, default=None, metavar="M_PER_MIN",
        help="컨베이어 초기 속도 [m/분]. 생략하면 기본 단계값. 키(, .)로도 바꿀 수 있고, "
             "여기 준 값은 단계 목록에 끼워 넣는다.",
    )
    parser.add_argument(
        "--defect-ratio", type=float, default=0.2, metavar="0~1",
        help="벨트에 투입되는 화물 중 불량품 비율. 대기열에 원하는 종류가 없으면 "
             "순서대로 내보내므로 장기 평균으로만 맞는다. 불량품이 없는 환경에서는 무시된다.",
    )
    parser.add_argument(
        "--defect-pattern", type=str, default="burst", metavar="SUBSTR",
        help="이 문자열이 이름에 들어간 화물을 불량품으로 본다 (예: corn_can_burst).",
    )
    parser.add_argument(
        "--spacing", type=float, default=None, metavar="M",
        help="화물 사이 간격 [m]. 입구 근처에 이 거리 안으로 화물이 있으면 투입을 미룬다.",
    )
    parser.add_argument(
        "--belt-jitter", type=float, default=0.15, metavar="0~1",
        help="벨트 속도를 기준의 ±이 비율 안에서 천천히 흔든다 (0=일정 속도, 기본 0.15). "
             "실물 컨베이어의 부하 변동을 흉내 내 데이터에 속도 변주를 넣는다.",
    )
    parser.add_argument(
        "--batch", type=int, default=0, metavar="N",
        help="0 보다 크면 배치(정적) 모드 — 벨트를 세우고 캔 N개를 무작위 위치에 "
             "놓는다. train 은 다 치우면 재배치, test 는 정상 캔을 모두 담으면 "
             "라운드 종료(trio_done) 후 전체 초기화. 0 이면 기존 연속 투입.",
    )
    parser.add_argument(
        "--arm-seed", type=int, default=0, metavar="1~5",
        help="task2 test 전용 — 작업자 팔이 들어오는 자리를 시드로 고정한다. "
             "1~5 는 배터리와 발전기 **사이** 통로를 좌우로 훑는 다섯 지점 "
             "(y -0.30/-0.20/-0.10/0.00/+0.10)이고, 높이는 운반 높이를 "
             "가로막는 0.30 으로 전부 같다. 0 이면 초기화마다 y 를 무작위로 "
             "뽑는다.",
    )
    parser.add_argument(
        "--grip-force", type=float, default=25.0, metavar="NM",
        help="그리퍼 관절 힘 상한 [Nm]. USD 기본은 finger_joint 16.5 / 링키지 5.0 이고 "
             "링키지 쪽이 실제 병목이다. 올리면 무거운 물체를 잡지만 폐루프 링키지가 "
             "불안정해져 그리퍼가 분해될 수 있다.",
    )
    parser.add_argument(
        "--can-mass", type=float, default=0.0, metavar="KG",
        help="화물 질량을 이 값으로 덮어쓴다. 0 이면 에셋 원본(0.35~0.5kg)을 쓴다.",
    )
    parser.add_argument(
        "--no-ros", action="store_true",
        help="ROS 2 노드를 띄우지 않는다. 기본은 켬 — Isaac Sim 이 rclpy 를 번들하고 "
             "있어 시스템 ROS 설치 없이 시뮬레이션 프로세스 안에서 돈다.",
    )
    parser.add_argument(
        "--ros-namespace", type=str, default="/franka",
        help="ROS 토픽 네임스페이스.",
    )
    parser.add_argument(
        "--no-fabric",
        action="store_true",
        help="USD Fabric 비활성화(진단용). 끄면 직접 쓴 위치가 렌더러에 전달되지 않아 "
             "화면이 멈춘 것처럼 보인다.",
    )
    return parser
