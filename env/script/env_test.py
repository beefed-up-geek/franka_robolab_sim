# SPDX-License-Identifier: Apache-2.0
"""컨베이어 pick-and-place 환경 — 동작 확인용 기본 환경.

블록이 컨베이어를 타고 흘러오고, 브라우저에서 키보드로 Franka 를 조작해 집는다.
앞으로 만들 실험 환경들의 템플릿이기도 하다 — 이 파일을 복사해 태스크와 기본값만
바꾸면 새 환경 스크립트가 된다.

실행:
    ./scripts/sim_start.sh env_test          # 컨테이너에서 백그라운드 기동
    # 또는 컨테이너 안에서 직접
    /workspace/isaaclab/_isaac_sim/python.sh env/script/env_test.py --headless

파일 순서가 중요하다. Isaac Sim(Kit)은 AppLauncher 로 앱을 띄운 뒤에야
isaaclab/robolab 모듈을 import 할 수 있어서, 아래 순서를 지켜야 한다.
    cv2 → sys.path 설정 → 인자 파싱 → AppLauncher → franka_env.runner
"""
# isort: skip_file
import sys
from pathlib import Path

import cv2  # noqa: F401  — isaaclab 보다 먼저 import 해야 한다. 지우지 말 것.

# env/src 를 import 경로에 넣는다. 컨테이너 PYTHONPATH 에도 들어 있지만,
# 저장소를 다른 곳에 두고 직접 실행할 때를 위해 여기서도 보장한다.
SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from isaaclab.app import AppLauncher  # noqa: E402
from franka_env.cli import build_parser  # noqa: E402

parser = build_parser(
    description="컨베이어 pick-and-place 환경 (테스트용 기본 환경)",
    task="ConveyorPickPlaceTask",
    camera="behind",
    conveyor="script",
)
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
args_cli.enable_cameras = True          # 카메라 렌더 없이는 브라우저 스트리밍이 불가능하다

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 앱이 뜬 뒤에만 import 할 수 있다.
from franka_env.runner import run  # noqa: E402

if __name__ == "__main__":
    try:
        run(args_cli, simulation_app)
    except Exception:
        sys.exit(1)
