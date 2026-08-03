# SPDX-License-Identifier: Apache-2.0
"""텔레오퍼레이션 전용 시점.

RoboLab 이 제공하는 HeadCameraCfg 는 쿼터니언 성분 순서가 어긋나 화면이 90° 돌아
나온다. 그리고 어차피 벤치마크용 카메라들은 정책 학습을 위한 배치라서, 사람이
키보드로 조작하기 좋은 시점과는 목적이 다르다.

여기 카메라는 로봇 뒤 왼쪽 위에서 작업면을 내려다본다. 정확히 뒤에 두면 팔이
물체를 가리므로 옆으로 비껴 놓되, 키 방향과 화면 방향이 크게 어긋나지 않는
선에서 타협한 위치다.

    카메라 right = (0.34, -0.94, 0)   → D 키(-Y)가 대체로 화면 오른쪽
    카메라 fwd   = (0.65,  0.23, -0.73) → W 키(+X)가 화면 안쪽
                                          Q 키(+Z)가 화면 위쪽

쿼터니언은 (eye=(-1.28, -0.66, 2.07), target=(0.50, 0, 0.15)) 에 대한 look-at
결과다. 시점을 바꾸려면 look-at 을 다시 계산해서 rot 을 통째로 갈아끼워야 한다 —
눈대중으로 성분을 만지면 위 축 대응이 깨진다.
"""
import isaaclab.sim as sim_utils
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass


@configclass
class TeleopFrontCameraCfg:
    """벨트 바깥쪽 위에서 작업면을 내려다보는 **고정** 카메라.

    궤도 카메라와 달리 아무도 옮기지 않으므로 아래 offset 이 곧 시점이다.
    eye=(1.55, 0, 0.88), target=(0.50, 0, 0.15) 의 look-at 결과 — 거리 1.28m·
    고도각 35°. 사람이 조작하며 쓰던 궤도 시점(반경 1.27, 위에서 내려다봄)을
    그대로 옮긴 것으로, 벨트 전체와 오른쪽의 담는 통까지 한 화면에 잡힌다
    (통 방위각 24° — 수평 반화각 26° 안쪽).

    예전에는 고도각 16° 였다: "더 올리면 창고 선반에 들어가 회색이 된다" 고
    기록했지만, 그건 거리 1.20 을 유지한 채 el 만 올렸을 때 얘기다. 같은
    고도라도 카메라를 조금 안쪽(x 1.65→1.55)으로 당기면 선반을 피한다 —
    사용자가 조작하던 궤도 카메라가 이 자리에서 멀쩡히 렌더된 것이 증거다.

    쿼터니언을 눈대중으로 만지면 위 축 대응이 깨진다. 시점을 바꾸려면 look-at 을
    다시 계산해 rot 을 통째로 갈아끼울 것 (관례: OpenGL look-at 행렬 → 쿼터니언의
    **켤레**. 기존 값이 그 부호였다).
    """

    teleop_front_camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/teleop_front_camera",
        # 보조 화면이라 메인 시점의 절반 크기로 렌더한다 — 인코딩 비용이 곧 제어율이다.
        height=270,
        width=480,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=5.5,
            focus_distance=28.0,
            horizontal_aperture=5.376,
            vertical_aperture=3.024,
        ),
        offset=TiledCameraCfg.OffsetCfg(
            pos=(1.55, 0.0, 0.88),
            rot=(0.626665, 0.327553, 0.327553, 0.626665),
            convention="opengl",
        ),
    )


@configclass
class TeleopViewCameraCfg:
    """자유 시점 3인칭 카메라 — behind / front 프리셋이 공유한다.

    아래 offset 은 첫 스텝에 덮어써지므로 사실상 초기값일 뿐이다. 실제 시점은
    config.VIEW_PRESETS 가 정하고, runner 가 매번 set_world_poses_from_view() 로
    옮긴다. 그래서 시점을 하나 더 만들 때 이 클래스를 복사할 필요가 없다.
    """

    teleop_view_camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/teleop_view_camera",
        # 스트림 폭(960)에 맞춰 렌더한다. 1280 으로 렌더한 뒤 960 으로 줄이면
        # 렌더·인코딩 비용만 늘고 화질 이득은 없다 — 제어율이 곧 조작감이다.
        height=540,
        width=960,
        data_types=["rgb"],
        # RoboLab 벤치마크 카메라의 focal_length=2.1 은 수평 화각 104° 라, 2.17m
        # 떨어진 이 위치에서는 테이블이 화면의 27% 밖에 안 된다. 5.5 로 좁히면
        # 화각 52°, 2.7m 지점 가시폭 2.64m 로 160cm 작업대가 화면의 61% 를 채운다.
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=5.5,
            focus_distance=28.0,
            horizontal_aperture=5.376,
            vertical_aperture=3.024,
        ),
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-1.2839, -0.6607, 2.0661),
            rot=(0.758850, 0.312726, -0.217665, -0.528177),
            convention="opengl",
        ),
    )
