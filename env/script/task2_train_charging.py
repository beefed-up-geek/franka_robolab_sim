# SPDX-License-Identifier: Apache-2.0
"""task2 train 환경 — 발전기 플러그를 배터리에 연결.

실행:
    ./scripts/sim_start.sh task2_train_charging

파일 순서가 중요하다 — task1 엔트리와 같은 구조.
"""
# isort: skip_file
import sys
from pathlib import Path

import cv2  # noqa: F401  — isaaclab 보다 먼저 import 해야 한다. 지우지 말 것.

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from isaaclab.app import AppLauncher  # noqa: E402
from franka_env.cli import build_parser  # noqa: E402

parser = build_parser(
    description="task2 발전기 플러그 연결 환경",
    task="Task2TrainTask",
    conveyor="none",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from franka_env.runner import run  # noqa: E402
from franka_env.world_assets import Task2ChargingWorldCfg  # noqa: E402

if __name__ == "__main__":
    try:
        run(args_cli, simulation_app, world_cfg=Task2ChargingWorldCfg)
    except Exception:
        sys.exit(1)
