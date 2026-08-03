# SPDX-License-Identifier: Apache-2.0
"""task3 평가 환경 — 팽창·파열된 불량품이 섞여 흐른다.

train 과 설비는 완전히 같고(씬이 _can_workcell.usda 를 공유한다) 흐르는 물건만
다르다. 정상품 5종에 그 짝인 파열품 5종을 더해 10종이 돌아간다.

짝을 맞춘 이유는 정책이 **결함 자체를** 보게 하기 위해서다. 불량품만 라벨이 다르면
그림만 외워도 골라낼 수 있다. 파열품은 부푼 뚜껑·찌그러진 옆면·뜯긴 구멍만 다르고
텍스처는 같다. 게다가 아래 뚜껑이 볼록해 벨트 위에서 비스듬히 기울기 때문에,
train 에서 보지 못한 파지 자세가 된다.

실행:
    ./scripts/sim_start.sh task3_test_pick_and_place_can

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
    description="task3 평가 환경 (불량품 포함)",
    task="Task3TestPickPlaceCanTask",
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
