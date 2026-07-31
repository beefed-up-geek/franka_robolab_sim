# SPDX-License-Identifier: Apache-2.0
"""브라우저 키보드로 Franka EEF 를 움직이는 텔레오퍼레이션 러너.

  브라우저(:8003) ──키입력──▶ TeleopState ──7차원 액션──▶ RoboLab env.step()
                  ◀──MJPEG──── 카메라 렌더 ◀─────────────┘

액션은 RoboLab 의 DroidRelIKActionCfg(상대 IK)를 그대로 쓴다. Isaac Lab 의
DifferentialIKController(DLS)가 IK 를 풀어주므로 MoveIt 같은 외부 IK 는 쓰지 않는다.

실행:
    /workspace/isaaclab/_isaac_sim/python.sh run_teleop.py --headless
"""
# isort: skip_file
import argparse
import cv2  # isaaclab 보다 먼저 import 해야 한다. 지우지 말 것.
import logging
import sys
import time
import traceback

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Franka 웹 키보드 텔레오퍼레이션")
parser.add_argument("--task", type=str, default="WhiteTablePickPlaceTask")
parser.add_argument("--stream-width", type=int, default=960, help="브라우저로 보낼 영상 가로 폭 [px]")
parser.add_argument(
    "--no-fabric",
    action="store_true",
    help="USD Fabric(flatcache) 비활성화. 리셋 후 관측이 stale 해지는 문제 진단용.",
)
parser.add_argument(
    "--camera",
    type=str,
    default="behind",
    choices=["behind", "head", "over_shoulder_left", "over_shoulder_right", "egocentric"],
    help="브라우저로 보낼 시점. 기본값 behind 만 키 방향과 화면 방향이 일치한다.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
args_cli.enable_cameras = True          # 카메라 렌더 없이는 스트리밍이 불가능하다
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""이하 Isaac Sim 기동 후에만 import 가능하다."""
import math  # noqa: E402
import torch  # noqa: E402
from pathlib import Path  # noqa: E402

import robolab.constants  # noqa: E402, F401
from robolab.core.environments.factory import auto_discover_and_create_cfgs, get_envs  # noqa: E402
from robolab.core.environments.runtime import create_env, end_episode  # noqa: E402
from robolab.core.observations.observation_utils import (  # noqa: E402
    generate_image_obs_from_cameras,
    generate_obs_cfg,
)
from robolab.robots.droid import (  # noqa: E402
    DroidCfg,
    DroidRelIKActionCfg,
    ProprioceptionObservationCfg,
    contact_gripper,
)
from robolab.variations.camera import (  # noqa: E402
    EgocentricMirroredCameraCfg,
    HeadCameraCfg,
    OverShoulderLeftCameraCfg,
    OverShoulderRightCameraCfg,
)
from robolab.variations.lighting import SphereLightCfg  # noqa: E402

from franka_teleop import config, safety  # noqa: E402
from franka_teleop.background import WarehouseBackgroundCfg  # noqa: E402
from franka_teleop.camera import TeleopBehindCameraCfg  # noqa: E402
from franka_teleop.state import TeleopState  # noqa: E402
from franka_teleop.web_server import start_in_thread  # noqa: E402

robolab.constants.VERBOSE = False
robolab.constants.RECORD_IMAGE_DATA = False

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger("teleop")

TASKS_DIR = str(Path(__file__).resolve().parent / "tasks")

CAMERAS = {
    "behind": TeleopBehindCameraCfg,                    # 로봇 뒤 위 — 키 방향과 화면 방향이 일치
    "head": HeadCameraCfg,                              # RoboLab 정면 시점 (화면이 90° 돌아간다)
    "over_shoulder_left": OverShoulderLeftCameraCfg,
    "over_shoulder_right": OverShoulderRightCameraCfg,
    "egocentric": EgocentricMirroredCameraCfg,          # 1인칭 — 작업면만 보인다
}


def register_env(task: str, camera_cfg) -> None:
    """이 저장소의 태스크를 상대 IK 액션 + Droid 로봇으로 등록한다.

    RTX 3090(24GB) 한 장이라 RoboLab 권장(48GB)에 못 미친다. 부가 센서를 빼고
    뷰포트 카메라 하나만 남겨 VRAM 을 아낀다 (실측 9.5GB).
    """
    ViewportObsCfg = generate_image_obs_from_cameras([camera_cfg])
    ObservationCfg = generate_obs_cfg({
        "proprio_obs": ProprioceptionObservationCfg(),
        "viewport_cam": ViewportObsCfg(),
    })
    auto_discover_and_create_cfgs(
        task_dir=TASKS_DIR,
        tasks=task,
        pattern="*.py",
        env_prefix="",
        env_postfix="Teleop",
        observations_cfg=ObservationCfg(),
        actions_cfg=DroidRelIKActionCfg(),
        robot_cfg=DroidCfg,
        camera_cfg=[camera_cfg],
        lighting_cfg=SphereLightCfg,
        background_cfg=WarehouseBackgroundCfg,
        contact_gripper=contact_gripper,
        dt=1 / (60 * 2),
        render_interval=8,
        decimation=8,
        seed=1,
    )


def first_image(group: dict) -> torch.Tensor | None:
    """관측 그룹에서 첫 번째 이미지 텐서 (N,H,W,C) 를 꺼낸다.

    카메라 관측 항목의 이름은 카메라 설정 클래스에서 동적으로 만들어지므로
    이름을 하드코딩하지 않고 모양으로 찾는다.
    """
    if not isinstance(group, dict):
        return group if isinstance(group, torch.Tensor) and group.ndim == 4 else None
    for value in group.values():
        if isinstance(value, torch.Tensor) and value.ndim == 4:
            return value
    return None


def encode_jpeg(image: torch.Tensor, width: int) -> bytes | None:
    """(N,H,W,C) RGB 텐서의 첫 환경 프레임을 JPEG 로 인코딩한다."""
    frame = image[0]
    if frame.dtype != torch.uint8:                      # float(0~1) 로 오는 경우
        frame = (frame.clamp(0, 1) * 255).to(torch.uint8)
    array = frame.detach().cpu().numpy()
    if array.shape[2] == 4:                             # RGBA → RGB
        array = array[:, :, :3]
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)

    if width and array.shape[1] > width:
        height = int(array.shape[0] * width / array.shape[1])
        array = cv2.resize(array, (width, height), interpolation=cv2.INTER_AREA)

    ok, buffer = cv2.imencode(".jpg", array, [int(cv2.IMWRITE_JPEG_QUALITY), config.STREAM_JPEG_QUALITY])
    return buffer.tobytes() if ok else None


def camera_sensor_name(camera_cfg) -> str | None:
    """카메라 설정 클래스에서 센서 이름(=env.scene 의 키)을 찾아낸다.

    RoboLab 은 설정 클래스의 속성 이름을 그대로 센서 이름으로 쓴다
    (generate_image_obs_from_cameras 와 같은 규칙).
    """
    from isaaclab.sensors import TiledCameraCfg

    instance = camera_cfg()
    for attr in dir(instance):
        if attr.startswith("_"):
            continue
        if isinstance(getattr(instance, attr, None), TiledCameraCfg):
            return attr
    return None


def screen_to_world(delta: list[float], azimuth: float) -> list[float]:
    """화면 기준 이동 델타를 월드 기준으로 돌린다.

    카메라를 궤도로 돌려도 "W 는 화면 안쪽, D 는 화면 오른쪽" 이 유지되게 하려면
    키 입력을 월드축이 아니라 카메라 방위각 기준으로 해석해야 한다. 안 그러면
    시점을 180° 돌렸을 때 W 가 화면 앞쪽으로 오는 꼴이 된다.

    방위각 az 는 target→eye 방향이므로 화면 안쪽(eye→target)은 -az 방향이다.
    """
    fwd = (-math.cos(azimuth), -math.sin(azimuth))     # 화면 안쪽
    left = (math.sin(azimuth), -math.cos(azimuth))     # 화면 왼쪽
    forward_amt, left_amt, up_amt = delta[0], delta[1], delta[2]
    return [
        forward_amt * fwd[0] + left_amt * left[0],
        forward_amt * fwd[1] + left_amt * left[1],
        up_amt,
        delta[3], delta[4], delta[5],                  # 회전은 월드 고정
    ]


def log_scene_bounds() -> None:
    """로드된 스테이지에서 테이블과 창고의 실제 크기·위치를 재서 찍는다.

    손으로 계산한 USD scale 과 시뮬레이터가 실제로 쓰는 값이 어긋나면 바로
    드러나고, 창고 안에서 작업대를 어디에 놓을지 정할 때도 이 값을 본다.
    """
    try:
        import omni.usd
        from pxr import Usd, UsdGeom

        stage = omni.usd.get_context().get_stage()
        wanted = {"table": None, "background": None}
        for prim in stage.Traverse():
            name = prim.GetName()
            if name in wanted and wanted[name] is None:
                wanted[name] = prim
            if all(v is not None for v in wanted.values()):
                break

        for name, prim in wanted.items():
            if prim is None:
                print(f"[teleop] '{name}' 프림을 찾지 못했습니다.", flush=True)
                continue
            rng = (
                UsdGeom.Imageable(prim)
                .ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
                .ComputeAlignedRange()
            )
            lo, hi, size = rng.GetMin(), rng.GetMax(), rng.GetSize()
            label = "테이블 상판" if name == "table" else "창고"
            print(
                f"[teleop] {label}: X {size[0]:.2f}m Y {size[1]:.2f}m Z {size[2]:.2f}m  "
                f"범위 X[{lo[0]:.2f},{hi[0]:.2f}] Y[{lo[1]:.2f},{hi[1]:.2f}] Z[{lo[2]:.2f},{hi[2]:.2f}]",
                flush=True,
            )
    except Exception as exc:  # noqa: BLE001
        print(f"[teleop] 씬 크기 측정 실패: {exc}", flush=True)


def reset_episode(env, state, reason: str):
    """에피소드를 끝내고 새로 시작한다.

    RobolabEnv 는 정책 벤치마크용이라, 에피소드가 종료되면 env 를 리셋하지 않고
    "freeze" 시킨다 (robolab/core/environments/env.py). freeze 된 env 는

        def step(self, action):
            if self._frozen_envs.any():
                action[self._frozen_envs] = 0.0   # 액션을 0 으로 덮어쓴다

    이 되어 팔이 영영 움직이지 않고, 상태도 종료 시점에 멈춘 채로 남는다.
    사람이 계속 조작해야 하는 텔레오퍼레이션에서는 이 동작이 치명적이므로,
    RoboLab 이 제공하는 reset_eval_state() 로 freeze 플래그와 _has_stepped 를
    내려서 다음 reset() 이 정상 리셋 경로(super()._reset_idx)를 타게 해야 한다.

    그냥 env.reset() 만 부르면 _has_stepped=True 라서 리셋이 아니라 freeze 가
    일어난다 — 겉보기엔 팔이 홈으로 돌아가도 그 뒤로 조작이 먹지 않는다.
    """
    end_episode(env)          # 레코더 정리 — 안 부르면 기록이 무한히 쌓인다
    env.reset_eval_state()    # freeze 해제 (이게 핵심)
    obs, _ = env.reset()
    state.on_reset_done()
    print(f"[teleop] 리셋: {reason}", flush=True)
    return obs


def main() -> None:
    camera_cfg = CAMERAS[args_cli.camera]
    register_env(args_cli.task, camera_cfg)
    task_envs = get_envs(task=args_cli.task)
    if not task_envs:
        logger.error("환경을 찾지 못했습니다: %s — tasks/ 아래 태스크 이름을 확인하세요.", args_cli.task)
        simulation_app.close()
        return
    env_name = task_envs[0]
    print(f"[teleop] 환경: {env_name}  카메라: {args_cli.camera}", flush=True)

    state = TeleopState()
    start_in_thread(state)

    env, _ = create_env(env_name, num_envs=1, use_fabric=not args_cli.no_fabric)
    print(f"[teleop] use_fabric={not args_cli.no_fabric}", flush=True)
    obs, _ = env.reset()
    log_scene_bounds()

    # 궤도 카메라용 핸들. 이름을 못 찾으면 시점 고정으로 동작한다.
    cam_name = camera_sensor_name(camera_cfg)
    camera = env.scene[cam_name] if cam_name and cam_name in env.scene.keys() else None
    if camera is None:
        print(f"[teleop] 경고: 카메라 센서 '{cam_name}' 를 찾지 못해 시점 조작이 비활성화됩니다.", flush=True)
    cam_target = torch.tensor([config.CAM_TARGET], device=env.device, dtype=torch.float32)
    last_eye = None

    action = torch.zeros(1, 7, device=env.device)
    step = 0
    hz_mark, hz_step, hz = time.monotonic(), 0, 0.0

    # Isaac Sim(Kit)이 로깅 설정을 덮어써서 logger 출력이 사라진다.
    # 기동 판정에 쓰는 신호이므로 print 로 직접 찍는다 (scripts/teleop_start.sh 참고).
    print(f"[teleop] 준비 완료 — 브라우저에서 http://<서버주소>:{config.PORT} 로 접속하세요.", flush=True)

    while simulation_app.is_running():
        delta, gripper, reset = state.consume()

        if reset:
            obs = reset_episode(env, state, f"R 키 (step {step})")
            step = 0
            continue

        # 시점이 바뀐 경우에만 카메라를 옮긴다 — 매 스텝 쓰면 낭비다.
        if camera is not None:
            eye = state.camera_eye()
            if eye != last_eye:
                camera.set_world_poses_from_view(
                    torch.tensor([eye], device=env.device, dtype=torch.float32), cam_target
                )
                last_eye = eye

        proprio = obs.get("proprio_obs", {})
        ee_pos = proprio.get("ee_pos")

        d = torch.tensor(
            screen_to_world(delta, state.camera_azimuth()), device=env.device, dtype=action.dtype
        )
        warn = ""
        if ee_pos is not None:
            # 목표가 작업공간을 벗어나지 않도록 델타를 잘라낸다.
            clamped = safety.clamp_delta(ee_pos[0], d[:3])
            if not torch.allclose(clamped, d[:3], atol=1e-6):
                warn = "작업공간 경계 — 이 방향으로는 더 이동할 수 없습니다"
            d[:3] = clamped

        action[0, :6] = d
        action[0, 6] = gripper

        obs, _, term, trunc, _ = env.step(action)
        step += 1

        # 제어 주파수 측정 (1초 창)
        hz_step += 1
        now = time.monotonic()
        if now - hz_mark >= 1.0:
            hz = hz_step / (now - hz_mark)
            hz_mark, hz_step = now, 0

        image = first_image(obs.get("viewport_cam", {}))
        if image is not None:
            jpeg = encode_jpeg(image, args_cli.stream_width)
            if jpeg is not None:
                state.publish_frame(jpeg)

        proprio = obs.get("proprio_obs", {})
        ee = proprio.get("ee_pos")
        state.publish_telemetry(
            ee_x=float(ee[0, 0]) if ee is not None else None,
            ee_y=float(ee[0, 1]) if ee is not None else None,
            ee_z=float(ee[0, 2]) if ee is not None else None,
            step=step,
            hz=hz,
            warn=warn,
        )

        if bool(term.any()) or bool(trunc.any()):
            obs = reset_episode(env, state, f"에피소드 종료 (step {step})")
            step = 0

    end_episode(env)
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        logger.error("종료: %s", exc)
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
