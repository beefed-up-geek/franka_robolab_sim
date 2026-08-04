# SPDX-License-Identifier: Apache-2.0
"""씬에 얹는 월드 에셋 — 창고 배경과 컨베이어.

두 가지를 한 설정 클래스에 모은 이유가 있다. RoboLab 의 환경 팩토리는 씬 설정에
합쳐 줄 자리를 robot/camera/lighting/background 네 개만 열어두었고, 이 클래스는
그중 background 자리로 들어가 **베이스 클래스로 병합**된다. 즉 여기 선언한
AssetBaseCfg 는 모두 씬 엔티티로 스폰된다.

컨베이어를 씬 USD 안에 payload 로 넣지 않고 이쪽으로 뺀 것도 그래서다. RoboLab 이
씬 USD 를 통째로 스폰할 때 payload 안의 콜라이더는 PhysX 에 등록되지 않아 물체가
그대로 통과한다. 씬 엔티티로 따로 스폰하면 collision_props 를 명시할 수 있다.

--- 원래 주석 ---
Isaac Sim 기본 창고 배경.

RoboLab 기본 배경(HomeOfficeBackgroundCfg 등)은 방 사진이 담긴 HDR 돔 라이트라
지오메트리가 없다 — 바닥도 벽도 무한히 먼 이미지일 뿐이라 로봇이 허공에 뜬 것처럼
보인다. 여기서는 Isaac Sim 이 기본 제공하는 창고 USD 를 실제 지오메트리로 깔아
로봇이 공간 안에 놓이게 한다.

에셋: {ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd
      ("작은 창고, 선반 1개" — Simple_Warehouse 중 가장 기본 구성)
      같은 폴더에 warehouse_multiple_shelves / warehouse_with_forklifts /
      full_warehouse 도 있으니 WAREHOUSE_USD 만 바꾸면 교체된다.

에셋은 NVIDIA 클라우드에서 받아 /root/.cache/ov 에 캐시된다. 첫 실행만 느리다.

좌표 주의: 창고 USD 는 자기 바닥이 z=0 이다. 이 프로젝트는 테이블 상판이 z=0,
지면이 z=-0.697 이므로 창고를 그만큼 내려서 바닥을 맞춘다.
"""
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

# 이 프로젝트의 지면 높이 (씬 USDA 의 GroundPlane 과 같아야 한다)
GROUND_Z = -0.697

WAREHOUSE_USD = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd"

# 로봇은 원점에 고정이므로, 로봇을 옮기는 대신 창고를 밀어서 원점이 빈 바닥
# 위에 오도록 맞춘다. X/Y 값은 실제 렌더를 보고 정한 것이다 (아래 주석 참고).
WAREHOUSE_OFFSET = (0.0, 0.0, GROUND_Z)


# 컨베이어 배치 — 길이(800)가 월드 Y 를 향하도록 Z 축으로 90도 돌린다.
# 결과: Y [-0.40, 0.40], X [0.42, 0.62], 반송면 z=0.20
CONVEYOR_USD = str(Path(__file__).resolve().parents[2] / "asset" / "conveyor" / "conveyor.usda")
CONVEYOR_POS = (0.52, 0.0, 0.0)
CONVEYOR_ROT = (0.7071068, 0.0, 0.0, 0.7071068)


# 담을 통(grey_bin) 배치 — 실측 420 x 280 x 105mm, 원점이 바닥이라 z=0 에 놓으면
# 상판(z=0) 위에 그대로 앉는다. Z 로 90도 돌려 긴 변을 Y 로 눕히면 컨베이어
# (X 0.42~0.62)와 겹치지 않으면서 로봇 팔이 닿는 자리에 들어간다.
#   결과: X [0.12, 0.40], Y [0.37, 0.79]
BIN_USD = str(Path(__file__).resolve().parents[2] / "asset" / "fixtures" / "grey_bin.usd")
BIN_POS = (0.26, 0.58, 0.0)
BIN_ROT = (0.7071068, 0.0, 0.0, 0.7071068)


@configclass
class WorldAssetsCfg:
    """창고 배경 + 컨베이어. RoboLab 의 background_cfg 자리로 넣는다."""

    conveyor = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/conveyor",
        spawn=sim_utils.UsdFileCfg(
            usd_path=CONVEYOR_USD,
            # 이걸 명시해야 콜라이더가 확실히 잡힌다.
            # 정적 콜라이더만 있으므로 activate_contact_sensors 는 켜지 않는다.
            # 켜면 "강체가 없다"며 스폰이 실패한다.
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=CONVEYOR_POS, rot=CONVEYOR_ROT),
    )

    warehouse = AssetBaseCfg(
        prim_path="/World/background",
        spawn=sim_utils.UsdFileCfg(usd_path=WAREHOUSE_USD),
        init_state=AssetBaseCfg.InitialStateCfg(pos=WAREHOUSE_OFFSET),
    )

    # 창고 자체 조명만으로는 작업면이 어두워서 은은한 돔 라이트를 얹는다.
    # visible_in_primary_ray=False 라 배경으로는 보이지 않고 빛만 보탠다.
    fill_light = AssetBaseCfg(
        prim_path="/World/fill_light",
        spawn=sim_utils.DomeLightCfg(
            intensity=180.0,
            color=(0.85, 0.87, 0.92),
            visible_in_primary_ray=False,
        ),
    )


@configclass
class CanSortingWorldCfg(WorldAssetsCfg):
    """창고 + 컨베이어에 담을 통을 더한 구성 (task3 환경들이 쓴다).

    통을 씬 USD 에 payload 로 넣지 않고 여기서 별도 엔티티로 스폰한다. RoboLab 이
    씬 USD 를 통째로 스폰할 때 payload 안의 정적 콜라이더가 PhysX 에 등록되지
    않아서, 담은 물건이 통을 그대로 통과해 버리기 때문이다. 컨베이어를 이쪽으로
    뺀 것과 같은 이유다.
    """

    bin = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/grey_bin",
        spawn=sim_utils.UsdFileCfg(
            usd_path=BIN_USD,
            # 명시해야 콜라이더가 확실히 잡힌다.
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=BIN_POS, rot=BIN_ROT),
    )


# ── task1: 공구 건네주기 ────────────────────────────────────────────────
# 작업자 캐릭터는 NVIDIA 공식 People 에셋을 S3 에서 직접 참조한다 (도구 뷰어
# 렌더에서 원격 참조가 동작함을 확인했다). 정지 자세 오버라이드는 추후 단계.
# 로컬 래퍼 — S3 원본을 참조하며 SkelAnimation 으로 "오른팔 내밀기" 자세를 얹는다.
# 원본을 그대로 쓰면 T포즈다. 래퍼는 tools/gen_pose.py 가 굽는다.
WORKER_USD = str(Path(__file__).resolve().parents[2] / "asset" / "fixtures"
                 / "worker_posed.usda")
# 도면 v3: 작업자 (0.50, -1.05), 바닥 z=-0.70. 캐릭터의 로컬 전방은 -Y 다 —
# Z+90 을 줬더니 +x 를 바라봤다(실측). -y_local → +y_world 는 Z 180° 다.
WORKER_POS = (0.50, -1.05, -0.70)
WORKER_ROT = (0.0, 0.0, 0.0, 1.0)

@configclass
class Task1HandoverWorldCfg:
    """창고 + 작업자 + 손바닥 받침. 컨베이어·통은 없다."""

    warehouse = AssetBaseCfg(
        prim_path="/World/background",
        spawn=sim_utils.UsdFileCfg(usd_path=WAREHOUSE_USD),
        init_state=AssetBaseCfg.InitialStateCfg(pos=WAREHOUSE_OFFSET),
    )

    fill_light = AssetBaseCfg(
        prim_path="/World/fill_light",
        spawn=sim_utils.DomeLightCfg(
            intensity=180.0,
            color=(0.85, 0.87, 0.92),
            visible_in_primary_ray=False,
        ),
    )

    worker = AssetBaseCfg(
        prim_path="/World/worker",
        spawn=sim_utils.UsdFileCfg(usd_path=WORKER_USD),
        init_state=AssetBaseCfg.InitialStateCfg(pos=WORKER_POS, rot=WORKER_ROT),
    )

    # 손바닥 받침은 두지 않는다. 핸드오버 판정은 받침 안착이 아니라 "공구가
    # 노란 테이프(y=-0.40)를 넘어 작업자 구역에 들어오면 성공 후 초기화" 로
    # 간다 — 차렷 자세가 기본이라 받침만 공중에 떠 보이는 문제도 없어진다.
