# SPDX-License-Identifier: Apache-2.0
"""task3 학습 환경 — 정상품만 흐른다.

정상 통조림 5종만 벨트를 타고 흘러온다. 높이가 32~58mm 로 제각각이라 파지 높이를
매번 맞춰야 하고, 그만큼 시연 데이터에 변화가 생긴다. 결함이 있는 물건은 하나도
나오지 않으므로, 여기서 모은 시연으로 학습한 정책은 파열품을 본 적이 없다.

실행:
    ./scripts/sim_start.sh task3_train_pick_and_place_can

파일 순서가 중요하다. Isaac Sim(Kit)은 AppLauncher 로 앱을 띄운 뒤에야
isaaclab/robolab 모듈을 import 할 수 있어서, 아래 순서를 지켜야 한다.
    cv2 → sys.path 설정 → 인자 파싱 → AppLauncher → franka_env.runner
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
    description="task3 학습 환경 (정상품만)",
    task="Task3TrainPickPlaceCanTask",
    view="behind",
    conveyor="script",
)
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
args_cli.enable_cameras = True          # 카메라 렌더 없이는 브라우저 스트리밍이 불가능하다

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 앱이 뜬 뒤에만 import 할 수 있다.
from franka_env.runner import run  # noqa: E402
from franka_env.world_assets import CanSortingWorldCfg  # noqa: E402

if __name__ == "__main__":
    try:
        # 창고·컨베이어에 더해 담을 통(grey_bin)까지 스폰한다.
        run(args_cli, simulation_app, world_cfg=CanSortingWorldCfg)
    except Exception:
        sys.exit(1)
