# SPDX-License-Identifier: Apache-2.0
"""환경 실행 스크립트가 공통으로 쓰는 인자 정의.

이 모듈은 Isaac Sim 을 띄우기 **전에** import 된다. 그래서 isaaclab·robolab·torch
같은 것을 절대 import 하면 안 된다 — argparse 만 쓴다.

환경마다 기본값이 다를 수 있으므로 `build_parser(task=..., camera=...)` 로 기본값을
바꿔 쓴다. 실제 환경 구성은 env/script 의 각 스크립트가 정한다.
"""
from __future__ import annotations

import argparse

CAMERA_CHOICES = ("behind", "head", "over_shoulder_left", "over_shoulder_right", "egocentric")
CONVEYOR_MODES = ("script", "force", "surface")


def build_parser(
    description: str,
    task: str,
    camera: str = "behind",
    conveyor: str = "script",
) -> argparse.ArgumentParser:
    """환경 실행 스크립트용 인자 파서를 만든다.

    Args:
        description: `--help` 에 나올 설명.
        task: 기본 태스크 이름 (env/src/tasks 의 클래스 이름).
        camera: 기본 시점.
        conveyor: 기본 컨베이어 구동 방식.
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--task", type=str, default=task, help="env/src/tasks 의 태스크 클래스 이름")
    parser.add_argument(
        "--camera",
        type=str,
        default=camera,
        choices=CAMERA_CHOICES,
        help="브라우저로 보낼 시점. behind 만 키 방향과 화면 방향이 일치한다.",
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
             "force=외력으로 마찰 흉내 — 외력이 반영되지 않아 동작하지 않는다. "
             "surface=PhysX 표면 속도 — 콜라이더가 깨져 동작하지 않는다.",
    )
    parser.add_argument(
        "--no-fabric",
        action="store_true",
        help="USD Fabric 비활성화(진단용). 끄면 직접 쓴 위치가 렌더러에 전달되지 않아 "
             "화면이 멈춘 것처럼 보인다.",
    )
    return parser
