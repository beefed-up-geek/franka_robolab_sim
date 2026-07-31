# SPDX-License-Identifier: Apache-2.0
"""Isaac Sim 기본 창고 배경.

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


@configclass
class WarehouseBackgroundCfg:
    """Isaac Sim Simple_Warehouse 를 배경 지오메트리로 깐다."""

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
