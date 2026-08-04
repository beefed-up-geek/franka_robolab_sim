# SPDX-License-Identifier: Apache-2.0
"""task1 환경 — 공구 건네주기.

실행:
    ./scripts/sim_start.sh task1_tool_handover

파일 순서가 중요하다. Isaac Sim(Kit)은 AppLauncher 로 앱을 띄운 뒤에야
isaaclab/robolab 모듈을 import 할 수 있다 (task3 엔트리와 같은 구조).
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
    description="task1 공구 건네주기 환경",
    task="Task1HandoverTask",
    conveyor="none",           # 벨트가 없다 — 컨베이어 로직 전체 비활성
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 앱이 뜬 뒤에만 import 할 수 있다.
from franka_env.runner import run  # noqa: E402
from franka_env.world_assets import Task1HandoverWorldCfg  # noqa: E402

if __name__ == "__main__":
    try:
        run(args_cli, simulation_app, world_cfg=Task1HandoverWorldCfg)
    except Exception:
        sys.exit(1)
