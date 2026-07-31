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
CONVEYOR_USD = str(Path(__file__).resolve().parents[1] / "assets" / "conveyor" / "conveyor.usda")
CONVEYOR_POS = (0.52, 0.0, 0.0)
CONVEYOR_ROT = (0.7071068, 0.0, 0.0, 0.7071068)


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
