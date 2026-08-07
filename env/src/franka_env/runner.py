# SPDX-License-Identifier: Apache-2.0
"""시뮬레이션 메인 루프.

Isaac Sim(Kit)은 `AppLauncher` 로 앱을 띄운 **뒤에야** isaaclab/robolab 모듈을
import 할 수 있다. 그래서 역할을 둘로 나눴다.

  env/script/*.py   인자를 읽고 앱을 띄운다 (환경마다 하나씩)
  이 모듈           앱이 뜬 뒤 import 되어 환경을 구성하고 루프를 돈다

환경을 새로 만들 때는 env/script 에 스크립트를 하나 더 두고 `run()` 에 넘길 인자만
바꾸면 된다 — 이 모듈은 건드릴 필요가 없다.

  브라우저(:8003) ──키입력──▶ TeleopState ──7차원 액션──▶ RoboLab env.step()
                  ◀──MJPEG──── 카메라 렌더 ◀─────────────┘

액션은 RoboLab 의 DroidRelIKActionCfg(상대 IK)를 그대로 쓴다. Isaac Lab 의
DifferentialIKController(DLS)가 IK 를 풀어주므로 MoveIt 같은 외부 IK 는 쓰지 않는다.
"""
# isort: skip_file
from __future__ import annotations

import math
import time
import traceback
from pathlib import Path

import cv2
import torch

from isaaclab.utils import configclass

import robolab.constants  # noqa: F401
from robolab.core.environments.factory import auto_discover_and_create_cfgs, get_envs
from robolab.core.environments.runtime import create_env, end_episode
from robolab.core.observations.observation_utils import (
    generate_image_obs_from_cameras,
    generate_obs_cfg,
)
from robolab.robots.droid import (
    DroidCfg,
    DroidRelIKActionCfg,
    ProprioceptionObservationCfg,
    WristCameraCfg,
    contact_gripper,
)
from robolab.variations.lighting import SphereLightCfg

from franka_env import config, safety
from franka_env.camera import (TeleopFrontCameraCfg, TeleopTopCameraCfg,
                               TeleopViewCameraCfg)
from franka_env import conveyor as conveyor_mod
from franka_env.conveyor import Conveyor
from franka_env.grasp import GraspSolver
from franka_env.ros_node import RosBridge
from franka_env.state import TeleopState
from franka_env.web_server import start_in_thread
from franka_env.world_assets import WorldAssetsCfg
from franka_env.verlet_wire import Task2Wires
from franka_env.worker_arm import ArmIntruder

robolab.constants.VERBOSE = False
robolab.constants.RECORD_IMAGE_DATA = False

# 태스크 정의는 env/src/tasks 에 있다 (이 파일 기준 ../tasks).
TASKS_DIR = str(Path(__file__).resolve().parents[1] / "tasks")

# 세 화면을 항상 함께 내보낸다. 브라우저는 /stream/<이름> 으로 골라 받는다.
#
#   view   궤도 카메라 — 드래그·줌으로 사람이 돌린다. --view 로 시작 자세만 고른다.
#   front  고정 정면 — 화물의 옆모습(부푼 뚜껑)이 실루엣으로 보인다.
#   wrist  그리퍼 붙박이 — RoboLab 기본 손목 카메라(DroidCfg 가 이미 만든다)
#
# RoboLab 의 head/over_shoulder/egocentric 선택지는 뺐다. 궤도 코드가 매 프레임 자세를
# 덮어썼기 때문에 그것들을 골라도 **위치는 그대로고 렌즈만 바뀌었다** — DROID 의 그
# 배치가 아니어서 이름이 거짓말이었다. 필요하면 VIEW_PRESETS 에 시점을 더한다.
STREAM_VIEWS = ("view", "front", "wrist")

# 손가락 링크가 base_link 에서 이보다 멀어지면 링키지가 터진 것으로 본다 [m].
EXPLODE_SPREAD_M = 0.25
EXPLODE_STEPS = 3       # 이만큼 연속으로 벌어져야 폭주로 본다

# 폭주가 되풀이될 때 리셋 강도를 올리는 기준.
#
# 실측에서 같은 좌표(0.52, -0.263, 0.229)에서 13회 연속 터졌다. 리셋이 팔만
# 되돌리고 손가락을 파고든 화물은 그 자리에 남는데, 정책이 결정론적이라 똑같이
# 접근해 똑같이 터지기 때문이다. 두 번째부터는 씬 전체를 되돌려 그 되풀이를 끊는다.
EXPLODE_ESCALATE = 2          # 이 횟수째 연속 폭주부터 full 리셋
EXPLODE_REPEAT_WINDOW = 600   # 직전 폭주가 이 스텝 안이면 "연속" 으로 본다


@configclass
class DroidTunedCfg(DroidCfg):
    """접촉 안정화만 더한 Droid — 그리퍼 드라이브는 **건드리지 않는다.**

    finger_joint 드라이브는 USD 에 stiffness 100 / damping 0.0002 / maxForce 16.5 로
    튜닝되어 있고 링키지는 mimic joint(고유진동수 1e6)로 묶여 있다. 여기에 damping 10
    (원래 값의 5만 배)을 얹었더니 그리퍼가 폭주해 팔에서 분리됐다 — 두 번. 드라이브는
    USD 값이 정답이다.

    무는 힘이 모자라 보이면 원인은 드라이브가 아니라 **물체 질량**이다. RoboLab 공식
    물체는 전부 0.02kg 인데 HOPE 캔은 0.18~0.4kg 이라, 씬 쪽에서 질량을 낮춰 맞춘다
    (task3_*.usda 의 physics:mass 오버라이드 참고).

    USD 의 finger_joint 드라이브는 stiffness 100 · **damping 0.0002** · maxForce 16.5 다.
    감쇠가 사실상 0 이라 위치 드라이브가 감쇠 없는 스프링이 되고, 캔을 문 채 진동한다.
    실측에서 접촉력이 매 스텝 1.2N ↔ 0N 으로 깜빡였고, 팔이 들어 올리며 가속하는
    순간 캔이 튕겨 나가고 손가락은 빈 채로 완전 폐쇄(0.785rad)까지 닫혔다.

    그래서 **damping 만** 올린다. stiffness 를 같이 올렸을 때(1000, 200) 두 번 다
    링키지가 폭주해 그리퍼가 팔에서 분리됐다 — mimic joint 로 묶인 폐루프라
    강성 증가에 매우 민감하다. 감쇠는 에너지를 빼는 방향이라 그 위험이 없다.
    """

    def __post_init__(self):
        post = getattr(super(), "__post_init__", None)
        if post is not None:
            post()
        g = self.robot.actuators["gripper"]
        g.stiffness = 100.0      # USD 와 동일 — **건드리면 폭주한다** (실측 2회)
        # 무는 힘. USD 기본값 16.5 는 RoboLab 자체 물체(0.02kg 블록) 기준이고,
        # HOPE 통조림은 0.18~0.40kg 로 최대 20배 무겁다. 마찰 2.0 에서 0.4kg 을
        # 버티려면 손가락 수직력이 1.0N 필요한데 실측 접촉력이 1.0~1.3N 이라
        # 여유가 거의 없어 조금만 흔들려도 미끄러졌다.
        #
        # 강성(100)은 그대로 두고 **상한만** 올린다. 강성을 올렸을 때는 두 번 다
        # 링키지가 폭주해 그리퍼가 팔에서 분리됐지만, 상한은 스프링 특성을 바꾸지
        # 않아 그 위험이 없다.
        g.effort_limit = 60.0
        # 유일하게 남기는 오버라이드. USD 의 damping 0.0002 는 사실상 감쇠 0 이라
        # 위치 드라이브가 감쇠 없는 스프링이 되어 캔을 문 채 진동하고, 들어 올리는
        # 순간 튕겨 나간다(실측: 접촉력이 매 스텝 1.2N ↔ 0N 으로 깜빡였다).
        # 감쇠는 에너지를 빼는 방향이라 강성과 달리 링키지를 불안정하게 하지 않는다.
        g.damping = 2.0


# ── 환경 등록 ────────────────────────────────────────────────────────────
def register_env(task: str, world_cfg) -> None:
    """태스크를 상대 IK 액션 + Droid 로봇으로 등록한다.

    RTX 3090(24GB) 한 장이라 RoboLab 권장(48GB)에 못 미친다. 부가 센서를 빼고
    뷰포트 카메라 하나만 남겨 VRAM 을 아낀다 (실측 9.5GB).

    손목 카메라는 씬에 **또** 스폰하면 안 된다. WristCameraCfg 는 관측 그룹에 이름만
    노출하는 래퍼이고 실제 센서는 DroidCfg 가 만든다 — 씬에 다시 넣으면 로봇 프림이
    생기기 전에 스폰을 시도해 "Unable to find source prim path" 로 죽는다.
    """
    scene_cams = [TeleopViewCameraCfg, TeleopFrontCameraCfg, TeleopTopCameraCfg]
    ViewportObsCfg = generate_image_obs_from_cameras(
        [TeleopViewCameraCfg, TeleopFrontCameraCfg, TeleopTopCameraCfg,
         WristCameraCfg]
    )
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
        robot_cfg=DroidTunedCfg,
        camera_cfg=scene_cams,
        lighting_cfg=SphereLightCfg,
        background_cfg=world_cfg,
        contact_gripper=contact_gripper,
        dt=1 / (60 * 2),
        render_interval=8,
        decimation=8,
        seed=1,
    )


# ── 관측 · 영상 ──────────────────────────────────────────────────────────
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

    ok, buffer = cv2.imencode(
        ".jpg", array, [int(cv2.IMWRITE_JPEG_QUALITY), config.STREAM_JPEG_QUALITY]
    )
    return buffer.tobytes() if ok else None


def gripper_spread(robot) -> float:
    """base_link 에서 손가락 링크까지의 최대 거리 [m].

    폐루프 링키지가 발산하면 링크들이 서로 밀려나 그리퍼가 분해되는데
    (Robotiq 2F-85 의 알려진 문제, IsaacSim #494), 이 거리가 그 신호다.
    정상은 0.15m 안쪽이다.
    """
    try:
        names = list(robot.data.body_names)
        pos = robot.data.body_pos_w[0]
        base = pos[names.index("base_link")]
        return max(
            float(torch.linalg.norm(pos[i] - base))
            for i, n in enumerate(names) if "finger" in n.lower()
        )
    except Exception:                                      # noqa: BLE001
        return 0.0


def _grip_status(robot, finger_idx, contact_sensors, env, exploded=False, **extra) -> dict:
    """파지 진단이 붙은 상태 요약.

    finger_q  손가락 관절각 [rad]. 0=열림, pi/4(0.785)=완전히 닫힘. 캔을 물면
              그 사이 어딘가에서 멈춘다 — 0.785 에 도달했다면 **아무것도 안 문 것**이다.
    contact   그리퍼-물체 접촉력 크기 [N]. 0 이면 손가락이 캔에 닿지도 않았다.
    exploded  **디바운스된** 폭주 판정 — 메인 루프의 연속 카운터를 그대로 싣는다.
              원시 spread 로 매번 다시 판정하면 안 된다: 손상된 링키지는 팔이
              가속할 때 spread 가 한두 스텝 0.25 를 스치는데, 그걸 그대로 내보냈더니
              수집기가 시도마다 즉시 포기해 83회 연속 실패했다(실제 폭주는 1회).
    """
    out = dict(extra)
    # spread 원시값은 진단용으로 계속 싣는다 (정상 0.15m 안쪽, IsaacSim #494).
    _spread = gripper_spread(robot)
    out["grip_spread"] = round(_spread, 4)
    out["exploded"] = bool(exploded)
    if finger_idx is not None:
        out["finger_q"] = round(float(robot.data.joint_pos[0, finger_idx]), 4)
    forces = {}
    for name, sensor_key in contact_sensors.items():
        try:
            data = env.scene[sensor_key].data
            # force_matrix_w 가 **그 객체와의** 접촉력이다. net_forces_w 는 센서
            # 이름과 무관한 그리퍼 전체 합이라, 이걸 내보내면 캔 하나를 무는 순간
            # 모든 키에 같은 힘이 찍힌다 — 수집기의 "지금 목표를 물었나" 판정과
            # 컨베이어의 "이 캔은 쥐어져 있나" 판정이 전부 그 값에 속았다.
            f = data.force_matrix_w
            f = f[0] if f is not None else data.net_forces_w[0]
            forces[name] = round(float(torch.linalg.norm(f).item()), 3)
        except Exception:                                  # noqa: BLE001
            continue
    if forces:
        out["contact"] = {k: v for k, v in forces.items() if v > 0.001}
    return out


def image_by_name(group, name: str) -> torch.Tensor | None:
    """관측 그룹에서 이름이 name 으로 시작하는 이미지 텐서를 꺼낸다.

    RoboLab 은 카메라 설정 클래스의 **속성 이름**으로 관측 항목을 만드는데, 접두사나
    접미사가 붙을 수 있어 정확히 일치하지 않는다. 그래서 부분 일치로 찾는다.
    """
    if not isinstance(group, dict):
        return None
    for key, value in group.items():
        if name in key and isinstance(value, torch.Tensor) and value.ndim == 4:
            return value
    return None


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


# ── 기동 시 진단 ─────────────────────────────────────────────────────────
def log_physics_scene() -> None:
    """PhysX 씬 설정을 찍는다.

    NVIDIA 공식 컨베이어 테스트는 enableGPUDynamics=False, broadphase=MBP 를
    전제로 한다. 표면 속도가 안 먹을 때 여기부터 확인하면 된다
    (실제로는 이 값이 맞아도 안 먹었다 — franka_env/conveyor.py 참고).
    """
    try:
        import omni.usd
        from pxr import PhysxSchema, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Scene):
                continue
            api = PhysxSchema.PhysxSceneAPI.Get(stage, prim.GetPath())

            def val(getter):
                try:
                    attr = getter()
                    return attr.Get() if attr else None
                except Exception:
                    return None

            print(
                f"[physx] {prim.GetPath()} "
                f"gpu_dynamics={val(api.GetEnableGPUDynamicsAttr)} "
                f"broadphase={val(api.GetBroadphaseTypeAttr)} "
                f"solver={val(api.GetSolverTypeAttr)} "
                f"ccd={val(api.GetEnableCCDAttr)}",
                flush=True,
            )
    except Exception as exc:  # noqa: BLE001
        print(f"[physx] 조회 실패: {exc}", flush=True)


def log_scene_bounds() -> None:
    """로드된 스테이지에서 테이블과 창고의 실제 크기·위치를 재서 찍는다.

    손으로 계산한 USD scale 과 시뮬레이터가 실제로 쓰는 값이 어긋나면 바로
    드러난다. 치수를 주장하기 전에 이 출력을 볼 것.
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
                print(f"[env] '{name}' 프림을 찾지 못했습니다.", flush=True)
                continue
            rng = (
                UsdGeom.Imageable(prim)
                .ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
                .ComputeAlignedRange()
            )
            lo, hi, size = rng.GetMin(), rng.GetMax(), rng.GetSize()
            label = "테이블 상판" if name == "table" else "창고"
            print(
                f"[env] {label}: X {size[0]:.2f}m Y {size[1]:.2f}m Z {size[2]:.2f}m  "
                f"범위 X[{lo[0]:.2f},{hi[0]:.2f}] Y[{lo[1]:.2f},{hi[1]:.2f}] "
                f"Z[{lo[2]:.2f},{hi[2]:.2f}]",
                flush=True,
            )
    except Exception as exc:  # noqa: BLE001
        print(f"[env] 씬 크기 측정 실패: {exc}", flush=True)


# ── 에피소드 ─────────────────────────────────────────────────────────────
def restore_robot(env) -> bool:
    """로봇 관절·루트 상태를 시뮬레이터에 직접 덮어쓴다 (hard 이상).

    그리퍼가 터진 뒤에는 env.reset() 만으로 복구되지 않는다. 리셋은 관절
    **목표**를 되돌릴 뿐, 폐루프가 발산하며 밀려난 링크의 실제 자세와 속도는
    그대로 남는다 — 실측에서 리셋 후에도 같은 자리에서 8회 연속 다시 터졌다.
    """
    try:
        r = env.scene["robot"]
        r.write_joint_state_to_sim(r.data.default_joint_pos.clone(),
                                   torch.zeros_like(r.data.default_joint_vel))
        r.write_root_pose_to_sim(r.data.default_root_state[:, :7].clone())
        r.write_root_velocity_to_sim(torch.zeros_like(r.data.default_root_state[:, 7:]))
        r.reset()
        return True
    except Exception as exc:                                   # noqa: BLE001
        print(f"[env] 로봇 강제 초기화 실패: {exc}", flush=True)
        return False


def restore_rigid_objects(env) -> int:
    """씬의 모든 강체를 기본 자세로 되돌린다 (full 전용).

    화물이 통 안에 쌓였든, 대기 자리에 숨어 있든, 그리퍼에 끼어 있든 상관없이
    씬 USD 가 정한 배치로 돌아온다. 이것이 "프로세스를 다시 띄운 것과 같은
    상태" 의 핵심이다.

    외력을 0 으로 덮어쓰는 것도 여기서 함께 한다. force 모드의 벨트가 매 스텝
    거는 힘은 **다음에 다시 쓸 때까지 남아 있어서**, 자세만 되돌리면 초기화
    직후 화물이 제자리에서 옆으로 밀려 나간다.

    Returns:
        되돌린 강체 수.
    """
    from isaaclab.assets import RigidObject

    n = 0
    origins = env.scene.env_origins
    for name in env.scene.keys():
        obj = env.scene[name]
        if not isinstance(obj, RigidObject):
            continue
        try:
            root = obj.data.default_root_state.clone()
            # default_root_state 는 환경 로컬 기준이다. env_origins 를 더하지
            # 않으면 환경이 여러 개일 때 전부 원점으로 겹쳐 놓인다.
            root[:, :3] += origins
            obj.write_root_pose_to_sim(root[:, :7])
            obj.write_root_velocity_to_sim(torch.zeros_like(root[:, 7:]))
            zero = torch.zeros((obj.num_instances, 1, 3), device=env.device)
            obj.set_external_force_and_torque(forces=zero, torques=zero, is_global=True)
            obj.reset()
            n += 1
        except Exception as exc:                               # noqa: BLE001
            print(f"[env] {name} 초기화 실패: {exc}", flush=True)
    return n


def reset_episode(env, state, belt, reason: str, level: str = config.RESET_DEFAULT):
    """에피소드를 끝내고 새로 시작한다.

    level 은 config.RESET_LEVELS — soft / hard / full. 강도가 올라갈수록 되돌리는
    범위가 넓어진다 (config 의 설명 참고). full 이면 프로세스를 다시 띄운 것과
    같은 상태가 되므로, 어떤 상황에서도 심을 죽일 필요가 없다.

    RobolabEnv 는 정책 벤치마크용이라, 에피소드가 종료되면 env 를 리셋하지 않고
    "freeze" 시킨다 (robolab/core/environments/env.py). freeze 된 env 는

        def step(self, action):
            if self._frozen_envs.any():
                action[self._frozen_envs] = 0.0   # 액션을 0 으로 덮어쓴다

    이 되어 팔이 영영 움직이지 않고, 상태도 종료 시점에 멈춘 채로 남는다.
    사람이 계속 조작해야 하는 환경에서는 치명적이므로, RoboLab 이 제공하는
    reset_eval_state() 로 freeze 플래그와 _has_stepped 를 내려서 다음 reset() 이
    정상 리셋 경로(super()._reset_idx)를 타게 해야 한다.

    그냥 env.reset() 만 부르면 _has_stepped=True 라서 리셋이 아니라 freeze 가
    일어난다 — 겉보기엔 팔이 홈으로 돌아가도 그 뒤로 조작이 먹지 않는다.
    """
    if level not in config.RESET_LEVELS:
        level = config.RESET_DEFAULT

    end_episode(env)          # 레코더 정리 — 안 부르면 기록이 무한히 쌓인다
    env.reset_eval_state()    # freeze 해제 (이게 핵심)

    if level in ("hard", "full") and restore_robot(env):
        print("[env] 로봇 관절 상태를 강제로 초기화했습니다.", flush=True)

    if level == "full":
        n = restore_rigid_objects(env)
        belt.reinit()         # 대기열 + 회수·투입 장부까지 0 으로
        print(f"[env] 화물 {n}개를 씬 기본 자세로 되돌렸습니다.", flush=True)
    else:
        belt.on_reset()       # 대기열은 비워야 화물이 벨트로 돌아온다

    obs, _ = env.reset()
    # full 은 env.reset() **뒤에** 한 번 더 덮어쓴다. RobolabEnv 가 리셋 경로에서
    # 물체 자세에 무엇을 하는지(랜덤화 여부)가 태스크 설정에 달려 있어, 앞에서만
    # 쓰면 그 결과에 지워질 수 있다. 두 번 쓰는 비용은 강체 몇 개 분량이라 무시할
    # 수준이고, "기본 자세로 돌아온다" 는 보장이 훨씬 값지다.
    if level == "full":
        restore_rigid_objects(env)
    state.on_reset_done(level)
    print(f"[env] 리셋({level}): {reason}", flush=True)
    return obs


# ── 메인 루프 ────────────────────────────────────────────────────────────
def run(args, simulation_app, world_cfg=None) -> None:
    """환경을 만들고 텔레오퍼레이션 루프를 돈다.

    Args:
        args: env/script 의 엔트리포인트가 파싱한 인자.
        simulation_app: AppLauncher 가 만든 앱 핸들.
        world_cfg: 씬에 얹을 월드 에셋 설정. 환경마다 다르므로 실행 스크립트가
            정한다 (창고 배경, 컨베이어, 담을 통 등). 생략하면 기본 구성.
    """
    try:
        _run(args, simulation_app, world_cfg or WorldAssetsCfg)
    except Exception as exc:  # noqa: BLE001
        print(f"[env] 종료: {exc}", flush=True)
        traceback.print_exc()
        simulation_app.close()
        raise


def _run(args, simulation_app, world_cfg) -> None:
    # 궤도 카메라(view) + 고정 정면(front) 은 씬에 스폰하고, 손목은 DroidCfg 가
    # 이미 만들어 두었으므로 관측 그룹에만 얹는다.
    view_preset = args.view
    register_env(args.task, world_cfg)
    task_envs = get_envs(task=args.task)
    if not task_envs:
        print(
            f"[env] 환경을 찾지 못했습니다: {args.task} — env/src/tasks 아래 "
            "태스크 이름을 확인하세요.",
            flush=True,
        )
        simulation_app.close()
        return
    env_name = task_envs[0]
    print(f"[env] 환경: {env_name}", flush=True)

    # 붙박이 카메라라도 궤도 상태는 "behind" 로 둔다 — 키 방향 해석(screen_to_world)이
    # 방위각을 쓰기 때문에, 시점이 고정이어도 기준 방위각은 있어야 한다.
    state = TeleopState(belt_mpm=args.belt_speed, view=view_preset or "behind")
    start_in_thread(state)

    # Fabric 은 CPU 물리에서도 켜 두어야 한다. 껐더니 write_root_pose_to_sim 으로
    # 쓴 위치가 렌더러에 전달되지 않아서, 물리 상태는 움직이는데 화면은 그대로인
    # 상태가 됐다(계측 숫자만 보고 동작한다고 착각하기 쉬운 함정이다).
    use_fabric = not args.no_fabric
    env, _ = create_env(
        env_name, device=args.physics_device, num_envs=1, use_fabric=use_fabric
    )
    print(
        f"[env] 물리={args.physics_device} use_fabric={use_fabric} "
        f"컨베이어={args.conveyor}",
        flush=True,
    )
    obs, _ = env.reset()
    log_scene_bounds()
    log_physics_scene()

    belt_mpm = (
        args.belt_speed
        if args.belt_speed is not None
        else config.BELT_SPEED_MPM[config.BELT_SPEED_DEFAULT_INDEX]
    )
    # 그리퍼 링키지의 힘 상한을 올린다.
    #
    # finger_joint 의 effort 를 16.5 → 60 으로 올려도 파지력이 늘지 않았다.
    # 힘이 mimic joint 로 전달되는데 right_outer_knuckle_joint 의 maxForce 가
    # **5 Nm** 라 거기서 막히기 때문이다. RoboLab 기본값은 0.02kg 블록 기준이고,
    # HOPE 통조림은 0.35~0.5kg 라 20배 무겁다.
    #
    # 강성이 아니라 힘 상한만 올리므로 스프링 특성은 그대로다 — 강성을 올렸을 때는
    # 두 번 다 링키지가 폭주해 그리퍼가 팔에서 분리됐다.
    try:
        import omni.usd
        _stage = omni.usd.get_context().get_stage()
        _raised = []
        for _prim in _stage.Traverse():
            if "Robotiq" not in str(_prim.GetPath()):
                continue
            # DriveAPI.Get() 은 쓸 수 없다 — 이 조인트들은 apiSchemas 가 비어 있고
            # drive 속성만 직접 박혀 있어서 스키마 조회가 실패한다. 속성을 직접 읽고 쓴다.
            _mf = _prim.GetAttribute("drive:angular:physics:maxForce")
            if not _mf:
                continue
            _v = _mf.Get()
            if _v is not None and 0 < _v < args.grip_force:
                _mf.Set(args.grip_force)
                _raised.append((_prim.GetName(), _v))
        print(f"[env] 그리퍼 관절 힘 상한 → {args.grip_force}Nm: {_raised}", flush=True)
    except Exception as _exc:                              # noqa: BLE001
        print(f"[env] 링키지 힘 조정 실패: {_exc}", flush=True)

    belt = Conveyor(
        env,
        belt_mpm / 60.0,
        mode=args.conveyor,
        defect_pattern=args.defect_pattern,
        defect_ratio=args.defect_ratio,
        spacing=args.spacing if args.spacing is not None else conveyor_mod.INLET_CLEARANCE,
        jitter=args.belt_jitter,
    )
    if belt.ready:
        print(
            f"[env] 컨베이어 준비 — 반송 대상 강체 {belt.item_count}개"
            f"(그중 순환 {belt.block_count}개, 불량품 {belt.defect_count}개), "
            f"속도 {belt_mpm:g} m/분, 간격 {belt.spacing:g}m, "
            f"불량품 비율 {belt.defect_ratio:.0%}",
            flush=True,
        )
    else:
        print("[env] 경고: 반송면 또는 화물을 찾지 못해 컨베이어가 동작하지 않습니다.", flush=True)

    # 궤도 카메라용 핸들. 궤도로 움직이는 것은 view 하나뿐이다.
    cam_name = camera_sensor_name(TeleopViewCameraCfg)
    camera = env.scene[cam_name] if cam_name and cam_name in env.scene.keys() else None
    print(f"[env] 화면 3개 송출: {', '.join(STREAM_VIEWS)} (궤도 시작 {view_preset})", flush=True)
    if camera is None:
        print(f"[env] 경고: 카메라 센서 '{cam_name}' 를 찾지 못해 시점 조작이 비활성화됩니다.", flush=True)
    cam_target = torch.tensor([config.CAM_TARGET], device=env.device, dtype=torch.float32)
    last_eye = None

    # 파지 자세 계산기. 홈 자세에서 손끝 오프셋을 재므로 첫 관측이 필요하다.
    _p = obs.get("proprio_obs", {})
    grasp = GraspSolver(env, _p.get("ee_pos"), _p.get("ee_quat")) \
        if _p.get("ee_pos") is not None else None

    # 화물 질량 덮어쓰기 (실험용). 0 이면 에셋 원본을 그대로 쓴다.
    if args.can_mass > 0:
        _n = 0
        for _name in belt.items:
            try:
                _obj = env.scene[_name]
                _m = _obj.root_physx_view.get_masses().clone()
                _m[:] = args.can_mass
                _obj.root_physx_view.set_masses(_m, torch.arange(_m.shape[0]))
                _n += 1
            except Exception as _exc:                      # noqa: BLE001
                print(f"[env] {_name} 질량 변경 실패: {_exc}", flush=True)
                break
        print(f"[env] 화물 질량 → {args.can_mass}kg ({_n}개)", flush=True)

    # 파지 진단 핸들. "왜 미끄러지나" 는 손가락이 실제로 닫혔는지와 접촉력이
    # 실제로 발생하는지를 봐야 갈린다 — 명령값(gripper_state)은 그 답이 못 된다.
    _robot = env.scene["robot"]
    try:
        _finger_idx = list(_robot.data.joint_names).index("finger_joint")
    except (ValueError, AttributeError):
        _finger_idx = None
        print("[env] 경고: finger_joint 를 찾지 못했습니다.", flush=True)
    _contact_sensors = {
        n.split("__", 1)[1]: n
        for n in env.scene.keys() if n.startswith("gripper__")
    }
    print(f"[env] 파지 진단 — finger_joint idx={_finger_idx}, "
          f"접촉센서 {sorted(_contact_sensors)}", flush=True)

    ros = RosBridge(state, namespace=args.ros_namespace, enabled=not args.no_ros)

    # 벨트 정지 판정에 쓰는 직전 스텝의 EEF 위치. 루프 뒷부분에서 갱신되므로
    # 첫 스텝에는 없다 — 6Hz 에서 한 스텝 지연은 문제되지 않는다.
    ee_pos = None
    explode_steps = 0          # 폭주 판정 디바운스 (한 프레임 튐과 구분)

    action = torch.zeros(1, 7, device=env.device)
    step = 0
    recycled = 0
    t2_attached = {}   # task2: 부착된 커넥터 -> 부착 스텝
    t2_weld = {}       # task2: 파지 용접된 커넥터 -> EEF 상대 오프셋
    t2_rope = None     # task2: 로프 링크 뷰 (기본 자세 복원용)
    t2_terms_pub = {}  # task2: ROS status 로 내보낼 단자 월드 좌표
    t2_rope_def = None
    t2_wires = None    # task2: 시각 전용 Verlet 전선 (verlet_wire.py)
    t2_arm = None      # task2 test: 작업자 팔 침입 (worker_arm.py)
    # 리셋해도 0 으로 돌아가지 않는 누적 스텝. 폭주가 "연속" 인지 판정하려면
    # 리셋을 건너 이어지는 시계가 필요하다.
    total_step = 0
    explode_count = 0
    last_explode = -EXPLODE_REPEAT_WINDOW
    hz_mark, hz_step, hz = time.monotonic(), 0, 0.0

    # Isaac Sim(Kit)이 로깅 설정을 덮어써서 logging 출력이 사라진다.
    # 기동 판정에 쓰는 신호라 print 로 직접 찍는다 (scripts/sim_start.sh 참고).
    print(f"[env] 준비 완료 — 브라우저에서 http://<서버주소>:{config.PORT} 로 접속하세요.", flush=True)

    while simulation_app.is_running():
        delta, gripper, reset = state.consume()

        if reset:
            t2_attached.clear()   # task2: 부착·용접 상태는 리셋과 함께 비운다
            t2_weld.clear()
            obs = reset_episode(env, state, belt, f"요청 (step {step})", level=reset)
            if t2_rope:
                # 로프는 reset_episode **뒤에** 되돌린다 — 리셋 도중(팔·물체가
                # 움직이는 사이) 로프를 텔레포트하면 제약 스냅으로 커넥터가
                # 튕겨 나간다 (실측: 테이블 밖 낙하). 복원 후 속도도 0 으로.
                t2_rope.set_world_poses(t2_rope_def[0].clone(), t2_rope_def[1].clone())
                t2_rope.set_velocities(torch.zeros((t2_rope.count, 6), device=t2_rope_def[0].device))
            if t2_wires:
                t2_wires.request_reset()
            if t2_arm:
                t2_arm.reset()
            ros.event("reset_done", step=step, level=reset, source="request")
            step = 0
            recycled = 0
            if reset == "full":
                # 전체 초기화는 폭주 이력도 지운다. 안 지우면 초기화 직후 한 번만
                # 터져도 곧바로 다시 full 로 올라간다.
                explode_count = 0
                last_explode = total_step - EXPLODE_REPEAT_WINDOW
            continue

        belt_change = state.consume_belt()
        if belt_change is not None:
            speed, on = belt_change
            belt.set_enabled(on)
            belt.set_speed(speed)

        # 그리퍼가 벨트 작업 구역에 들어와 있으면 벨트를 멈춘다.
        if ee_pos is not None:
            belt.update_hold(ee_pos[0])

        # 벨트에 얹힌 물체를 밀어 준다. env.step() 직전에 써야 이번 스텝에 반영된다.
        belt.drive()
        belt.drive_force()

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

        # 키보드 델타만 화면 기준 → 월드로 돌린다. ROS 델타는 이미 월드 기준이라
        # 여기에 섞으면 카메라 방위각만큼 돌아가 엉뚱한 방향으로 간다.
        world = screen_to_world(delta, state.camera_azimuth())
        ext = state.consume_external()
        if ext is not None:
            world = [w + e for w, e in zip(world, ext)]
        d = torch.tensor(world, device=env.device, dtype=action.dtype)
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
        total_step += 1

        # 그리퍼 폭주 감지 — 링키지가 터지면 그대로 두어도 복구되지 않으므로
        # 에피소드를 리셋한다. ROS 로 알려서 수집기가 그 구간을 버릴 수 있게 한다.
        _spread = gripper_spread(_robot)
        if _spread > EXPLODE_SPREAD_M:
            explode_steps += 1
            if explode_steps >= EXPLODE_STEPS:
                # 직전 폭주와 가까우면 "같은 자리에서 되풀이" 로 보고 강도를 올린다.
                if total_step - last_explode < EXPLODE_REPEAT_WINDOW:
                    explode_count += 1
                else:
                    explode_count = 1
                last_explode = total_step
                _level = "full" if explode_count >= EXPLODE_ESCALATE else "hard"
                print(f"[env] 그리퍼 폭주 감지 (spread {_spread:.3f}m, "
                      f"연속 {explode_count}회) — {_level} 리셋", flush=True)
                ros.event("gripper_explosion", step=step, spread=round(_spread, 4),
                          repeat=explode_count, level=_level)
                _ee = ee_pos[0] if ee_pos is not None else None
                obs = reset_episode(env, state, belt,
                                    f"그리퍼 폭주 (spread {_spread:.3f}m)", level=_level)
                # 리셋 **뒤에** 격리한다 — reset_episode 가 belt.on_reset() 으로
                # 대기열을 비우므로, 먼저 넣으면 그대로 지워진다.
                # full 은 화물을 전부 기본 자세로 되돌리므로 격리할 것이 없다.
                _moved = (belt.requeue_near(_ee)
                          if _ee is not None and _level != "full" else [])
                if _moved:
                    print(f"[env] 폭주 지점의 화물을 대기열로 되돌렸습니다: {_moved}",
                          flush=True)
                ros.event("reset_done", step=step, level=_level,
                          source="explosion", requeued=_moved)
                explode_steps = 0
                step = 0
                recycled = 0
                if _level == "full":
                    explode_count = 0
                continue
        else:
            explode_steps = 0

        # 출구를 지난 화물을 입구로 되돌려 흐름을 끊기지 않게 한다.
        recycled += belt.recycle()

        # task1(벨트 없음): 공구가 경계 테이프(y=-0.40)를 넘으면 **그 공구만**
        # 씬 기본 자세로 되돌린다 — 전달 판정이자 다음 시연 준비다. 정책은
        # 내려놓을 필요 없이 수평으로 들고 선을 넘기만 하면 된다.
        if belt.mode == "none" and "battery" not in belt.items:
            _crossed = []
            _origin = env.scene.env_origins[0]
            for _name in belt.items:
                _obj = env.scene[_name]
                if float(_obj.data.root_pos_w[0, 1] - _origin[1]) < -0.40:
                    _root = _obj.data.default_root_state.clone()
                    _root[:, :3] += env.scene.env_origins
                    _obj.write_root_pose_to_sim(_root[:, :7])
                    _obj.write_root_velocity_to_sim(torch.zeros_like(_root[:, 7:]))
                    _obj.reset()
                    _crossed.append(_name)
            if _crossed:
                print(f"[env] 경계 통과 — 초기화: {_crossed}", flush=True)
                ros.event("tool_crossed", step=step, tools=_crossed)

        # ── task2: 커넥터를 배터리 단자에 씌우면 부착(스냅·유지)하고, 둘 다
        #    부착되면 12스텝 뒤 커넥터를 초기 자세로 되돌린다. 단자 좌표는
        #    SAM3D 정점 측정값(배터리 로컬) — red→B(+), black→A(-).
        if belt.mode == "none" and "battery" in belt.items and "connector_red" in belt.items:
            # test 전용: 작업자 팔 침입 — 운반 중(커넥터 들림·미부착)에만
            # 진입한다. 양쪽 부착 완료면 즉시 후퇴 (완료 초기화 방해 금지).
            if "worker_arm" in belt.items:
                if t2_arm is None:
                    t2_arm = ArmIntruder(env.scene["worker_arm"])
                    print("[env] task2 작업자 팔 — 침입 상태기계 활성", flush=True)
                _carrying = False
                for _an in ("connector_red", "connector_black"):
                    if _an not in t2_attached:
                        if float(env.scene[_an].data.root_pos_w[0, 2]) > 0.12:
                            _carrying = True
                t2_arm.step(_carrying, len(t2_attached) == 2, ros, step)
            # 시각 전용 Verlet 전선 — 커넥터 글랜드에 핀 고정되어 따라온다
            if t2_wires is None:
                try:
                    import omni.usd
                    t2_wires = Task2Wires(omni.usd.get_context().get_stage())
                    print(f"[env] task2 전선 — Verlet 로프 "
                          f"{'활성' if t2_wires.ok else '프림 없음'}", flush=True)
                except Exception as _e:
                    t2_wires = False
                    print(f"[env] task2 전선 초기화 실패: {_e}", flush=True)
            if t2_wires:
                for _wname in ("connector_red", "connector_black"):
                    _wobj = env.scene[_wname]
                    t2_wires.step(_wname,
                                  _wobj.data.root_pos_w[0].tolist(),
                                  _wobj.data.root_quat_w[0].tolist())
            if t2_rope:
                # 전선이 상판 아래로 파고들지 않게 z 하한을 강제한다
                _rp, _rq = t2_rope.get_world_poses()
                if bool((_rp[:, 2] < 0.004).any()):
                    _rp[:, 2] = torch.clamp(_rp[:, 2], min=0.004)
                    t2_rope.set_world_poses(_rp, _rq)
            _bat = env.scene["battery"]
            _bp = _bat.data.root_pos_w[0]
            _bq = _bat.data.root_quat_w[0]
            # 상면 수직 렌더 판독으로 교정 — 두 포스트 모두 y=-0.05 모서리.
            # (-0.09,+0.053) 지점은 작은 캡(디코이)이라 제외했다.
            # 접촉 증거로 재교정 — 서보 하강 시 소켓 바닥이 실제로 얹힌 지점.
            # 부착 목표 = 단자 연장 포스트 상단 (배터리 로컬 z 0.208)
            # 검은 플러그는 빨간 플러그가 단자에 꽂힐 때까지 제자리 고정
            # (실무 규칙: + 먼저 — 작업 중 밀리거나 넘어지는 것도 막는다).
            # 자성 홀드와 같은 매 스텝 재기록이라 리셋과 간섭 없다.
            if "connector_red" not in t2_attached:
                _blk = env.scene["connector_black"]
                _broot = _blk.data.default_root_state.clone()
                _broot[:, :3] += env.scene.env_origins
                _blk.write_root_pose_to_sim(_broot[:, :7])
                _blk.write_root_velocity_to_sim(torch.zeros_like(_broot[:, 7:]))
            _TERMS = {"connector_red": (0.0962, -0.0546, 0.208),
                      "connector_black": (-0.0962, -0.0523, 0.208)}
            _SLEEVE = 0.015
            # 소켓이 속 빈 캡이라 포스트가 실제로 들어간다 — 바닥이 포스트
            # 상단 아래 6mm 이상 내려간(=씌워진) 순간에만 부착으로 본다.
            _R_XY = 0.016
            _INSERT = -0.006
            _w, _x, _y, _z = (float(_bq[0]), float(_bq[1]), float(_bq[2]), float(_bq[3]))
            for _name, _off in _TERMS.items():
                _obj = env.scene[_name]
                _ox, _oy, _oz = _off
                _tx = (1 - 2*(_y*_y + _z*_z))*_ox + 2*(_x*_y - _w*_z)*_oy + 2*(_x*_z + _w*_y)*_oz
                _ty = 2*(_x*_y + _w*_z)*_ox + (1 - 2*(_x*_x + _z*_z))*_oy + 2*(_y*_z - _w*_x)*_oz
                _tz = 2*(_x*_z - _w*_y)*_ox + 2*(_y*_z + _w*_x)*_oy + (1 - 2*(_x*_x + _y*_y))*_oz
                _term = (float(_bp[0]) + _tx, float(_bp[1]) + _ty, float(_bp[2]) + _tz)
                t2_terms_pub["pos" if _name == "connector_red" else "neg"] = [
                    round(_term[0], 4), round(_term[1], 4), round(_term[2], 4)]
                if _name in t2_attached:
                    # 자성 홀드 (사용자 요청 복원) — 부착된 커넥터를 단자 위에
                    # 스냅해 고정한다. 완료 초기화 때 함께 풀린다.
                    _ox2, _oy2, _oz2 = _off
                    _tx2 = (1 - 2*(_y*_y + _z*_z))*_ox2 + 2*(_x*_y - _w*_z)*_oy2 + 2*(_x*_z + _w*_y)*_oz2
                    _ty2 = 2*(_x*_y + _w*_z)*_ox2 + (1 - 2*(_x*_x + _z*_z))*_oy2 + 2*(_y*_z - _w*_x)*_oz2
                    _tz2 = 2*(_x*_z - _w*_y)*_ox2 + 2*(_y*_z + _w*_x)*_oy2 + (1 - 2*(_x*_x + _y*_y))*_oz2
                    _root = _obj.data.default_root_state.clone()
                    _root[:, 0] = float(_bp[0]) + _tx2
                    _root[:, 1] = float(_bp[1]) + _ty2
                    _root[:, 2] = float(_bp[2]) + _tz2 - _SLEEVE
                    _root[:, 3] = 1.0
                    _root[:, 4:7] = 0.0
                    _obj.write_root_pose_to_sim(_root[:, :7])
                    _obj.write_root_velocity_to_sim(torch.zeros_like(_root[:, 7:]))
                    continue
                _cp = _obj.data.root_pos_w[0]
                _dx = float(_cp[0]) - _term[0]
                _dy = float(_cp[1]) - _term[1]
                _dz = float(_cp[2]) - _term[2]
                if _dx*_dx + _dy*_dy < _R_XY*_R_XY and -0.022 < _dz < _INSERT:
                    t2_attached[_name] = step
                    _pol = "B(+)" if _name == "connector_red" else "A(-)"
                    print(f"[env] 부착 — {_name} → {_pol} 단자", flush=True)
                    ros.event("connector_attached", step=step, connector=_name)
            if len(t2_attached) == 2 and step - max(t2_attached.values()) > 12:
                for _name in ("connector_red", "connector_black"):
                    _obj = env.scene[_name]
                    _root = _obj.data.default_root_state.clone()
                    _root[:, :3] += env.scene.env_origins
                    _obj.write_root_pose_to_sim(_root[:, :7])
                    _obj.write_root_velocity_to_sim(torch.zeros_like(_root[:, 7:]))
                    _obj.reset()
                t2_attached.clear()
                if t2_rope:
                    # 로프도 함께 복원 — 안 하면 남은 장력이 방금 초기화된
                    # 커넥터를 끌어 넘어뜨린다 (실측)
                    t2_rope.set_world_poses(t2_rope_def[0].clone(), t2_rope_def[1].clone())
                    t2_rope.set_velocities(torch.zeros((t2_rope.count, 6), device=t2_rope_def[0].device))
                if t2_wires:
                    t2_wires.request_reset()
                print("[env] 충전 연결 완료 — 커넥터·로프 초기화", flush=True)
                ros.event("charging_done", step=step)

        # 제어 주파수 측정 (1초 창)
        hz_step += 1
        now = time.monotonic()
        if now - hz_mark >= 1.0:
            hz = hz_step / (now - hz_mark)
            hz_mark, hz_step = now, 0

        # 메인 화면은 매 스텝, 보조 화면은 드문드문. 세 장을 매번 인코딩하면
        # 인코딩만으로 제어율이 떨어진다.
        group = obs.get("viewport_cam", {})
        aux_turn = step % config.AUX_STREAM_STRIDE == 0
        ros_images: dict[str, bytes] = {}
        for view_name, sensor, width in (
            ("front", "teleop_front_camera", 480),
            ("top", "teleop_top_camera", 480),
            ("wrist", "wrist_cam", 480),
        ):
            if not aux_turn:
                continue
            aux = image_by_name(group, sensor)
            if aux is not None:
                aux_jpeg = encode_jpeg(aux, width)
                if aux_jpeg:
                    state.publish_frame(aux_jpeg, view_name)
                    ros_images[view_name] = aux_jpeg

        image = image_by_name(group, "teleop_view_camera")
        if image is not None:
            jpeg = encode_jpeg(image, args.stream_width)
            if jpeg is not None:
                state.publish_frame(jpeg, "view")
                ros_images["view"] = jpeg

        proprio = obs.get("proprio_obs", {})
        ee = proprio.get("ee_pos")
        # ── ROS ──────────────────────────────────────────────────────
        # 자세는 물리 원점 기준으로 맞춘다. env_origins 를 빼지 않으면 환경이
        # 여러 개일 때 좌표가 통째로 어긋난다.
        if ros.ok:
            origin = env.scene.env_origins[0]
            # 벨트 위 캔에 **진행 순서**를 매긴다. 출구에 가까울수록 0 이다.
            # 정책이 y 를 보고 직접 정렬하지 않아도 되게 하려는 것이고, 대기열에
            # 숨어 있는(상판 아래) 캔은 -1 이라 애초에 후보가 되지 않는다.
            on_belt_order = {}
            ranked = []
            for name in belt.items:
                pos = env.scene[name].data.root_pos_w[0] - origin
                if belt.is_on_belt(name, pos):
                    ranked.append((float(pos[1]), name))
            belt_order = [name for _, name in sorted(ranked, reverse=True)]
            for rank, name in enumerate(belt_order):
                on_belt_order[name] = rank

            objects, grasps = {}, {}
            for name in belt.items:
                o = env.scene[name]
                pos = o.data.root_pos_w[0] - origin
                objects[name] = (pos.tolist(), o.data.root_quat_w[0].tolist())
                if grasp is not None and grasp.ok:
                    # 손끝을 물체 중심에 두는 것이 목표다. 뚜껑 쪽은 파열 캔의
                    # 부푼 돔에 미끄러지고, 바닥 쪽은 벨트와 부딪힌다.
                    flange = grasp.flange_for(pos)
                    grasps[name] = {
                        "flange": flange.tolist(),
                        "quat": grasp.home_quat.tolist(),
                        "on_belt": name in on_belt_order,
                        "order": on_belt_order.get(name, -1),
                        "half_height": round(belt.half_height(name), 4),
                    }
            ros.publish(
                eef_pos=(ee_pos[0].tolist() if ee_pos is not None else None),
                eef_quat=(proprio.get("ee_quat")[0].tolist()
                          if proprio.get("ee_quat") is not None else None),
                gripper=gripper,
                objects=objects,
                grasps=grasps,
                belt_order=belt_order,
                images=ros_images,
                status=_grip_status(
                    _robot, _finger_idx, _contact_sensors, env,
                    exploded=explode_steps >= EXPLODE_STEPS,
                    hz=hz, step=step, recycled=recycled,
                    **{k: belt.status().get(k)
                       for k in ("binned", "off_belt", "queued", "belt_held")},
                    # 유효 속도(흔들림 반영)를 준다. 기준 속도를 주면 정책의 벨트
                    # 추종 피드포워드가 빨라진 구간에서 뒤처져 파지점이 밀린다.
                    belt_mpm=belt.current_mpm(),
                    terminals=(t2_terms_pub or None),
                ),
            )
            ros.spin()

        state.publish_telemetry(
            ee_x=float(ee[0, 0]) if ee is not None else None,
            ee_y=float(ee[0, 1]) if ee is not None else None,
            ee_z=float(ee[0, 2]) if ee is not None else None,
            step=step,
            hz=hz,
            recycled=recycled,
            **belt.status(),
            warn=warn,
        )

        if bool(term.any()) or bool(trunc.any()):
            obs = reset_episode(env, state, belt, f"에피소드 종료 (step {step})")
            step = 0
            recycled = 0

    end_episode(env)
    env.close()
    simulation_app.close()
