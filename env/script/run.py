# SPDX-License-Identifier: Apache-2.0
"""환경 공용 엔트리 — 환경 이름과 인자만 주면 어떤 환경이든 띄운다.

    ./scripts/sim_start.sh task2_train                     # 평소엔 이걸 쓴다
    python env/script/run.py task3_test --belt-speed 2.0   # (컨테이너 안)

어떤 환경이 있고 환경별 기본 인자가 무엇인지는 franka_env/envs.py 의
레지스트리가 정한다 — 예전처럼 환경마다 런처 파일을 복사하지 않는다.

파일 순서가 중요하다. Isaac Sim(Kit)은 AppLauncher 로 앱을 띄운 뒤에야
isaaclab/robolab 모듈을 import 할 수 있고, cv2 는 isaaclab 보다 먼저여야 한다.
"""
# isort: skip_file
import sys
from pathlib import Path

import cv2  # noqa: F401  — isaaclab 보다 먼저 import 해야 한다. 지우지 말 것.

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from franka_env.cli import build_parser  # noqa: E402
from franka_env.envs import ENVS  # noqa: E402


def _usage() -> str:
    lines = ["사용 가능한 환경:"]
    for name, spec in ENVS.items():
        lines.append(f"  {name:<12} {spec.description}")
    return "\n".join(lines)


if len(sys.argv) < 2 or sys.argv[1] not in ENVS:
    print(f"✗ 환경 이름이 필요합니다.\n{_usage()}", file=sys.stderr)
    sys.exit(2)

ENV_NAME = sys.argv[1]
SPEC = ENVS[ENV_NAME]
sys.argv = [sys.argv[0]] + sys.argv[2:]     # 환경 이름은 파서에 넘기지 않는다

from isaaclab.app import AppLauncher  # noqa: E402

parser = build_parser(
    description=f"{ENV_NAME} — {SPEC.description}",
    task=SPEC.task,
    conveyor=SPEC.conveyor,
)
parser.set_defaults(**SPEC.defaults)
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
args_cli.enable_cameras = True   # 카메라 렌더 없이는 브라우저 스트리밍이 불가능하다

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 앱이 뜬 뒤에만 import 할 수 있다.
from franka_env import world_assets  # noqa: E402
from franka_env.runner import run  # noqa: E402

if __name__ == "__main__":
    try:
        run(args_cli, simulation_app, world_cfg=getattr(world_assets, SPEC.world))
    except Exception:
        sys.exit(1)
