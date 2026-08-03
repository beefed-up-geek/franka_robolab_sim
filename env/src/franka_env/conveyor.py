# SPDX-License-Identifier: Apache-2.0
"""컨베이어 구동과 블록 순환.

## 왜 PhysX 표면 속도를 쓰지 않는가

처음에는 `PhysxSurfaceVelocityAPI`(PhysX 표면 속도)로 만들었다. 공식
`isaacsim.asset.gen.conveyor` 익스텐션도 같은 방식이다 — 소스를 보면
`RigidBodyAPI` → `CollisionAPI` → `PhysxSurfaceVelocityAPI` 를 차례로 적용한다.
즉 이건 우회로가 아니라 우리가 막힌 바로 그 API 다.

이 프로젝트 구성(Isaac Lab gym 워크플로 + GPU/Fabric 물리)에서는 동작하지 않는다.
두 가지가 겹친다.

1. 표면 속도를 적용하는 순간 **그 프림의 콜라이더가 무효화된다.** 블록이 벨트를
   그대로 통과해 테이블로 떨어졌다. Isaac Lab 이슈 #4561 에 같은 증상이 있다 —
   "the mere presence/activation of the PhysxConveyorAPI ... causes the collider
   to fail", GUI 에서는 되는데 gym 워크플로에서만 깨진다.
2. 표면 속도는 애초에 **CPU 물리 전용**이라 GPU/Fabric 과 양립하지 않는다.

디버깅 중 씬 파일에 직접 넣은 검증용 콜라이더는 멀쩡히 동작했는데, 그건 그 프림에만
표면 속도를 걸지 않았기 때문이었다. USD 합성(payload/reference/키네마틱)을 계속
바꿔 보느라 한참 헤맸지만 그쪽은 원인이 아니었다.

## 이 환경에서 무엇이 되고 안 되는가 (전부 실측)

| 방법 | 결과 |
|---|---|
| `PhysxSurfaceVelocityAPI` (정석·공식) | 콜라이더가 무효화되어 물체가 벨트를 통과 |
| └ GPU 물리 + 정적 콜라이더 | 통과 |
| └ GPU 물리 + 키네마틱 강체 | 통과 |
| └ CPU 물리 + 정적 콜라이더 | 통과 |
| └ CPU 물리 + 키네마틱 강체 (**NVIDIA 공식 테스트와 동일 구성**) | 통과 |
| `set_external_force_and_torque` | 반영되지 않음 (25 m/s^2 를 줘도 미동 없음) |
| `write_root_velocity_to_sim` | 반영되지 않음 (명령 후에도 vy=0) |
| `write_root_pose_to_sim` | **동작함** |

2026-08 재시도: NVIDIA 문서·포럼이 말하는 **전체 레시피**를 그대로 맞췄다 —
벨트를 키네마틱 강체로 승격, 물리 CPU, `use_fabric=False`, PhysX 씬의
GPU 다이내믹스 off, broadphase MBP. 이번엔 콜라이더가 살아남아 캔이 벨트를
통과하지는 않았지만, **화물이 전혀 움직이지 않았다** (8초에 Δy=0.0000).
즉 이 워크플로에서 표면 속도는 어떤 조합으로도 반송을 만들지 못한다.

표면 속도는 NVIDIA 공식 테스트(isaacsim.asset.gen.conveyor/tests/test_conveyor.py)와
같은 조건 — PhysX 씬이 gpu_dynamics=False, broadphase=MBP, solver=TGS 이고 벨트가
키네마틱 강체 — 을 모두 맞춘 뒤에도 실패했다. 같은 하드웨어(RTX 3090)·같은 워크플로
(Isaac Lab InteractiveSceneCfg 텔레오퍼레이션)에서 동일한 증상이 NVIDIA 포럼에도
보고되어 있고, NVIDIA 답변은 "GitHub 에 이슈를 올려라" 뿐이었다.

그래서 아래 방식은 게으른 선택이 아니라, 이 환경에서 실제로 작동하는 유일한
수단이다. 외력·속도가 왜 반영되지 않는지는 아직 규명하지 못했다.

## 그래서 어떻게 하는가

벨트는 **평범한 정적 콜라이더**로 두고, 벨트에 얹혀 있는 블록을 매 스텝 직접
전진시킨다.

속도만 써 주는 방법(`write_root_velocity_to_sim`)도 시도했지만 반영되지 않았다 —
명령 후에도 블록의 vy 가 계속 0 으로 측정됐다(sleep 문턱값을 0 으로 낮춰도 같음).
반면 **위치 쓰기(`write_root_pose_to_sim`)는 확실히 동작한다** — 회수 텔레포트가
그 경로로 잘 돌아간다. 그래서 위치를 speed × dt 만큼 적분해 전진시키고, 속도는
보조로 함께 써 준다.

부수 효과로 벨트 위에서는 X 와 Z 가 고정되어 블록이 옆으로 흐르거나 가라앉지
않는다. 벨트 위 블록은 물리적으로 밀치기보다 "실려 가는" 쪽에 가깝다.

로봇이 블록을 집어 든 순간에는 구동을 멈춰야 그리퍼와 싸우지 않는다. "벨트 높이에
얹혀 있을 때만" 속도를 주는 것으로 자연스럽게 해결된다 — 조금이라도 들어 올리면
z 가 범위를 벗어나 구동 대상에서 빠진다.
"""
from __future__ import annotations

import torch

# 벨트 콜라이더 프림 이름 (env/asset/conveyor/conveyor.usda)
BELT_PRIM_NAME = "belt_surface"

# 씬 배치 기준 — franka_env/world_assets.py 의 컨베이어 배치와 맞물린다.
BELT_X = 0.52          # 벨트 중심의 월드 X
BELT_TOP_Z = 0.200     # 반송면 높이
INLET_Y = -0.36        # 블록이 들어오는 쪽
OUTLET_Y = 0.36        # 이 지점을 넘으면 회수한다
BLOCK_HALF = 0.0225    # 45mm 정육면체의 절반

# 순환에서 빼 둘 물체. 벨트에 올릴 일이 없는 고정물(담는 그릇 등)을 적는다.
# 그 외에는 벨트 위에서 출구를 지나면 무엇이든 입구로 되돌린다.
NO_RECIRCULATE = frozenset({"bowl"})

# 물체가 벨트에 "얹혀 있다" 고 볼 범위. 물체마다 높이가 다르므로 기준 높이는
# 실측한 반높이로 물체별로 계산한다.
REST_Z_TOL = 0.015                   # 이보다 들리면 구동 대상에서 제외
ON_BELT_DX = 0.09                    # 벨트 폭 방향 허용치

# 화물 사이 간격. 입구 근처에 이 거리 안으로 다른 화물이 있으면 투입을 미루므로,
# 결과적으로 벨트 위 간격이 된다. 0.22 면 지름 70mm 통조림 사이에 150mm 쯤
# 빈 자리가 생긴다.
#
# 사용 구간이 0.72m 뿐이라 이 간격에서는 벨트에 3개까지만 올라간다. 그래서 종류를
# 여러 개 유지하려면 대기열이 필요하다 — 출구를 지난 화물은 테이블 아래
# STAGING_POS 에 숨겨 두었다가 입구가 비면 하나씩 다시 투입한다.
INLET_CLEARANCE = 0.22

# 담는 통. 여기 들어간 화물은 사라져 대기열로 돌아간다 — 안 그러면 통이 넘치고,
# 사람이 계속 조작하는 샌드박스에서 얼마 못 가 담을 곳이 없어진다.
# 위치와 크기는 스테이지에서 직접 재므로 world_assets.py 의 배치를 바꿔도 따라간다.
BIN_PRIM_NAME = "grey_bin"
BIN_INSET = 0.02        # 테두리에 걸친 것을 삼키지 않도록 안쪽으로 물린다 [m]

# 대기 중인 화물을 숨겨 두는 곳. 상판(z=0) 아래라 카메라에 잡히지 않는다.
# 중력에 떨어지지 않도록 대기하는 동안 매 스텝 이 자리에 다시 써 준다.
STAGING_POS = (BELT_X, INLET_Y, -0.30)

# 반송면보다 이만큼 아래로 내려가면 "벨트에서 떨어졌다" 고 본다 [m].
# 가장 납작한 참치캔의 반높이(16mm)보다 넉넉히 커야 정상 주행을 오인하지 않는다.
OFF_BELT_DZ = 0.06

# 그리퍼가 이 구역 안으로 들어오면 벨트를 멈춘다. 움직이는 캔을 무는 것은
# 상대 속도 때문에 실패율이 높은데, 로봇이 손을 뻗는 동안만 멈춰 주면 파지가
# 정지 상태 문제가 되어 훨씬 안정된다. 실제 물류 설비에서도 피킹 구간은
# 인덱싱(간헐 이송)으로 돌리는 것이 보통이다.
# 그리퍼가 **컨베이어 위 영역** 안에 있는 동안만 벨트를 멈춘다. 손이 그 영역을
# 벗어나는 순간 — 캔을 물고 통 쪽으로 빠져나가기 시작하면 — 곧바로 다시 돈다.
#
# 높이 조건은 두지 않는다. 예전에는 z<0.44 를 함께 봤는데, LIFT 가 끝나는 높이가
# 0.431 이라 캔을 든 뒤에도 계속 정지 상태로 남았다. 판정은 **평면 위치만** 본다.
HOLD_DX = 0.14          # 벨트 중심에서 이 거리 안 [m] (벨트 반폭 0.08)
HOLD_Y = (-0.42, 0.42)  # 벨트 길이 방향 범위 [m]

# 그리퍼가 "쥐고 있다" 고 볼 접촉력 [N]. 스치는 접촉과 가르는 값이다.
GRASP_CONTACT_N = 0.3

# 테이블 아래로 완전히 떨어진 물체는 어디에 있든 회수한다.
FALLEN_Z = -0.10

# 출구 너머 회수 판정 범위. X 는 벨트 연장선에서 크게 벗어나지 않아야 하고
# (그릇에 담긴 블록을 건드리지 않으려면 좁아야 한다), Z 는 로봇이 들어 올린
# 상태를 걸러내는 용도다.
RECOVER_DX = 0.13
RECOVER_DZ = 0.08

# force 모드 — 벨트 마찰을 힘으로 흉내 낸다.
# 목표 속도와의 차이에 비례해 힘을 주되, 실제 마찰이 낼 수 있는 최대 가속도로
# 자른다. 이 상한이 곧 "벨트가 물체를 얼마나 세게 끌 수 있는가" 이고, 넘으면
# 물체가 미끄러진다 — 실제 컨베이어와 같은 거동이다.
DRIVE_GAIN = 25.0       # [1/s] 속도 오차 → 가속도
DRIVE_ACCEL_MAX = 25.0  # [m/s^2] 벨트가 낼 수 있는 최대 가속도.
# 시뮬레이터 벨트면의 마찰(경우에 따라 PhysX 기본값 0.5 가 적용될 수 있다)을
# 확실히 이기도록 넉넉히 잡았다. μ=0.5 면 제동 가속도가 4.9 m/s^2 이므로
# 그보다 충분히 커야 물체가 실제로 끌린다.


class Conveyor:
    """벨트 위 블록을 구동하고, 출구를 지난 블록을 입구로 되돌린다."""

    def __init__(
        self,
        env,
        speed: float,
        mode: str = "surface",
        *,
        defect_pattern: str = "burst",
        defect_ratio: float = 0.2,
        spacing: float = INLET_CLEARANCE,
    ) -> None:
        """
        Args:
            defect_pattern: 이 문자열이 이름에 들어간 화물을 불량품으로 본다.
                태스크가 따로 알려 주지 않아도 되도록 이름 규칙으로 정한다
                (`corn_can_burst` 처럼 접미사를 붙여 두면 된다).
            defect_ratio: 투입되는 화물 중 불량품 비율 (0~1).
            spacing: 입구 근처에 이 거리 안으로 다른 화물이 있으면 투입을 미룬다.
        """
        self.env = env
        self.speed = speed
        self.enabled = True
        self.mode = mode
        self.defect_ratio = max(0.0, min(1.0, defect_ratio))
        self.spacing = spacing
        self._surface_apis: list = []
        self._binned = 0
        self._off_belt = 0
        self._held = False
        self._released = {True: 0, False: 0}   # 불량품 / 정상품 투입 수

        # env.scene 에는 블록 강체뿐 아니라 접촉 센서도 들어 있고, 센서 이름이
        # "block_0__block_1" 처럼 같은 접두사로 시작한다. 이름만 보고 고르면
        # 센서가 섞여 들어와 root_pos_w 를 읽다 죽는다. 타입으로 걸러야 한다.
        from isaaclab.assets import RigidObject

        # 벨트에 얹히면 **무엇이든** 실어 나른다. 블록만 골라내면 나중에 다른 물체를
        # 올렸을 때 가만히 있어서 컨베이어가 아니게 된다.
        self._items: list[str] = sorted(
            name
            for name in env.scene.keys()
            if isinstance(env.scene[name], RigidObject)
        )
        # 회수 대상 — 고정물만 뺀다. 실제 회수는 "벨트 위에서 출구를 지났을 때"
        # 만 일어나므로, 로봇이 집어 다른 곳에 둔 물체는 저절로 대상에서 빠진다.
        self._blocks: list[str] = [n for n in self._items if n not in NO_RECIRCULATE]
        self._half_height: dict[str, float] = self._measure_half_heights()
        # 입구가 빌 때까지 기다리는 화물. 순서대로 다시 투입된다.
        self._queue: list[str] = []
        # 그리퍼-화물 접촉 센서. 로봇이 쥔 화물을 벨트가 계속 붙잡지 않게 하려면
        # "지금 누가 만지고 있는가" 를 알아야 한다.
        self._grip_sensors = {
            n.split("__", 1)[1]: n
            for n in env.scene.keys() if n.startswith("gripper__")
        }
        self._defects = frozenset(n for n in self._blocks if defect_pattern in n)
        self._bin = self._measure_bin()
        self._belt_found = self._check_belt()
        if self.mode == "surface":
            self._apply_surface_velocity()
            self._disable_gpu_dynamics()

    def _measure_half_heights(self) -> dict[str, float]:
        """각 물체의 반높이를 스테이지에서 재 둔다.

        "벨트에 얹혀 있다" 를 판정하려면 물체 중심이 반송면에서 얼마나 위에 있어야
        하는지 알아야 하는데, 그 값은 물체 높이에 따라 다르다. 블록 크기를 상수로
        박아 두면 다른 물체를 올렸을 때 판정이 어긋난다.
        """
        result: dict[str, float] = {}
        try:
            import omni.usd
            from pxr import Usd, UsdGeom
        except ImportError:
            return result

        stage = omni.usd.get_context().get_stage()
        wanted = set(self._items)
        for prim in stage.Traverse():
            name = prim.GetName()
            if name not in wanted or name in result:
                continue
            rng = (
                UsdGeom.Imageable(prim)
                .ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
                .ComputeAlignedRange()
            )
            if rng.IsEmpty():
                continue
            result[name] = float(rng.GetSize()[2]) / 2.0
        return result

    def _measure_bin(self) -> tuple[float, float, float, float, float] | None:
        """담는 통의 월드 경계를 스테이지에서 재 둔다. (x0, x1, y0, y1, 테두리 z)

        통은 씬 USD 가 아니라 world_assets.py 가 별도로 스폰하므로 env.scene 에
        강체로 잡히지 않는다. 그래서 크기를 상수로 박지 않고 직접 잰다.
        """
        try:
            import omni.usd
            from pxr import Usd, UsdGeom
        except ImportError:
            return None

        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetName() != BIN_PRIM_NAME:
                continue
            rng = (
                UsdGeom.Imageable(prim)
                .ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
                .ComputeAlignedRange()
            )
            if rng.IsEmpty():
                continue
            lo, hi = rng.GetMin(), rng.GetMax()
            box = (
                float(lo[0]) + BIN_INSET, float(hi[0]) - BIN_INSET,
                float(lo[1]) + BIN_INSET, float(hi[1]) - BIN_INSET,
                float(hi[2]),
            )
            print(
                f"[bin] {prim.GetPath()} X[{box[0]:.2f},{box[1]:.2f}] "
                f"Y[{box[2]:.2f},{box[3]:.2f}] 테두리 z={box[4]:.3f}",
                flush=True,
            )
            return box

        print(f"[bin] '{BIN_PRIM_NAME}' 프림을 찾지 못해 통 회수를 끕니다.", flush=True)
        return None

    def _over_bin(self, x: float, y: float) -> bool:
        """통의 평면 범위 안인가 (높이 무관).

        통으로 떨어지는 중인 화물을 낙하 회수가 가로채지 않게 하려면 필요하다.
        떨어지는 동안 z 는 반송면보다 한참 아래를 지나는데, 아직 테두리 아래로는
        들어가지 않은 순간이 있어서 그때 대기열로 끌려갔다.
        """
        if self._bin is None:
            return False
        x0, x1, y0, y1, _ = self._bin
        return x0 < x < x1 and y0 < y < y1

    def _in_bin(self, x: float, y: float, z: float) -> bool:
        """통 안에 들어갔는가.

        테두리보다 **낮아야** 한다. 통 위를 지나가거나 그리퍼가 위쪽에 들고 있는
        동안에는 사라지지 않는다.
        """
        if self._bin is None:
            return False
        x0, x1, y0, y1, z_top = self._bin
        return x0 < x < x1 and y0 < y < y1 and z < z_top

    def set_defect_ratio(self, ratio: float) -> None:
        self.defect_ratio = max(0.0, min(1.0, ratio))

    @property
    def defect_count(self) -> int:
        return len(self._defects)

    # ── 벨트 ────────────────────────────────────────────────────────────
    def _check_belt(self) -> bool:
        """벨트 콜라이더가 제자리에 있는지 확인만 한다.

        여기서 PhysX 표면 속도를 걸면 안 된다 — 거는 순간 콜라이더가 죽는다
        (모듈 상단 설명 참고).
        """
        try:
            import omni.usd
            from pxr import Usd, UsdGeom, UsdPhysics
        except ImportError:
            return False

        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetName() != BELT_PRIM_NAME:
                continue
            rng = (
                UsdGeom.Imageable(prim)
                .ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
                .ComputeAlignedRange()
            )
            lo, hi = rng.GetMin(), rng.GetMax()
            print(
                f"[belt] {prim.GetPath()} collider={prim.HasAPI(UsdPhysics.CollisionAPI)} "
                f"top_z={hi[2]:.3f} X[{lo[0]:.2f},{hi[0]:.2f}] Y[{lo[1]:.2f},{hi[1]:.2f}]",
                flush=True,
            )
            return True

        print(f"[belt] '{BELT_PRIM_NAME}' 프림을 찾지 못했습니다.", flush=True)
        return False

    def _apply_surface_velocity(self) -> None:
        """반송면에 PhysX 표면 속도를 건다.

        정석이자 유일하게 "아무 물체나" 실어 나르는 방식이다. 마찰을 통해 전달되므로
        벨트에 닿기만 하면 블록이든 그릇이든 똑같이 움직인다. 대신 전제가 두 가지다 —
        벨트가 **키네마틱 강체**여야 하고, 물리가 **CPU** 여야 한다.
        """
        try:
            import omni.usd
            from pxr import Gf, PhysxSchema
        except ImportError:
            return

        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetName() != BELT_PRIM_NAME:
                continue
            # 공식 권장 순서: CollisionAPI → MeshCollisionAPI → RigidBodyAPI(키네마틱)
            # → PhysxSurfaceVelocityAPI. 정적 콜라이더로는 표면 속도가 전달되지
            # 않는다는 것이 NVIDIA 문서·포럼의 공통된 지적이라 여기서 키네마틱
            # 강체로 승격시킨다 (USDA 는 콜라이더만 갖고 있다).
            from pxr import UsdPhysics as _UsdPhysics

            rb = _UsdPhysics.RigidBodyAPI.Apply(prim)
            rb.CreateKinematicEnabledAttr(True)
            api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
            api.CreateSurfaceVelocityEnabledAttr(True)
            # 로컬 좌표로 줘야 컨베이어를 씬에서 어떻게 돌려 놓든 로컬 +X 가
            # 진행 방향이 된다.
            api.CreateSurfaceVelocityLocalSpaceAttr(True)
            api.CreateSurfaceVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            self._surface_apis.append(api)
        self._push_surface_speed()

    def _disable_gpu_dynamics(self) -> None:
        """PhysX 씬의 GPU 다이내믹스를 끈다 — 표면 속도는 CPU 전용이다."""
        try:
            import omni.usd
            from pxr import PhysxSchema, UsdPhysics
        except ImportError:
            return
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Scene):
                continue
            api = PhysxSchema.PhysxSceneAPI.Apply(prim)
            api.CreateEnableGPUDynamicsAttr(False)
            api.CreateBroadphaseTypeAttr("MBP")
            print(f"[belt] {prim.GetPath()} GPU 다이내믹스 off / MBP", flush=True)

    def _push_surface_speed(self) -> None:
        from pxr import Gf

        value = self.speed if self.enabled else 0.0
        for api in self._surface_apis:
            api.GetSurfaceVelocityAttr().Set(Gf.Vec3f(value, 0.0, 0.0))

    def on_reset(self) -> None:
        """에피소드 리셋 시 대기열을 비운다.

        비우지 않으면 리셋으로 화물이 제자리에 돌아가도 다음 스텝에서 다시
        대기 자리로 끌려 내려간다. 결국 벨트가 텅 비고 전부 상판 아래에 갇힌다.
        """
        self._queue.clear()

    def reinit(self) -> None:
        """컨베이어를 기동 직후 상태로 되돌린다 (전체 초기화).

        on_reset() 은 대기열만 비우지만, 이쪽은 **장부까지** 되돌린다. 회수 수와
        투입 수가 남아 있으면 초기화 후에도 통계가 이어져 "몇 개를 담았나" 가
        섞이고, 그걸 보고 성공률을 세는 수집기가 틀린 값을 남긴다. 불량품 투입
        쿼터도 함께 처음으로 돌아간다 (첫 투입이 다시 불량이 된다).

        화물 자세는 여기서 손대지 않는다 — 강체를 기본 자세로 되돌리는 일은
        씬 전체를 아는 runner.restore_rigid_objects 쪽이 한다. 여기서 함께
        건드리면 같은 프림에 두 번 쓰게 되어 어느 쪽이 이겼는지 알 수 없다.
        """
        self._queue.clear()
        self._binned = 0
        self._off_belt = 0
        self._held = False
        self._released = {True: 0, False: 0}

    def update_hold(self, ee_pos) -> bool:
        """그리퍼가 컨베이어 위 영역에 있으면 벨트를 멈춘다. 매 스텝 호출한다.

        영역을 벗어나는 순간 — 캔을 물고 통 쪽으로 나가기 시작하면 — 다시 돈다.
        """
        x, y = float(ee_pos[0]), float(ee_pos[1])
        self._held = abs(x - BELT_X) < HOLD_DX and HOLD_Y[0] < y < HOLD_Y[1]
        return self._held

    @property
    def held(self) -> bool:
        return self._held

    def set_speed(self, speed: float) -> None:
        self.speed = speed
        if self.mode == "surface":
            self._push_surface_speed()

    def set_enabled(self, on: bool) -> None:
        self.enabled = on
        if self.mode == "surface":
            self._push_surface_speed()

    @property
    def ready(self) -> bool:
        if self.mode == "surface":
            return self._belt_found and bool(self._surface_apis)
        return self._belt_found and bool(self._blocks)

    @property
    def block_count(self) -> int:
        return len(self._blocks)

    def half_height(self, name: str) -> float:
        """실측한 반높이. 파지 높이를 정할 때 쓴다."""
        return self._half_height.get(name, BLOCK_HALF)

    def is_on_belt(self, name: str, pos) -> bool:
        """지금 벨트에 얹혀 있는가 (로봇이 들고 있으면 거짓)."""
        return self._on_belt(name, float(pos[0]), float(pos[1]), float(pos[2]))

    @property
    def items(self) -> list[str]:
        """벨트가 실어 나를 수 있는 물체 이름. ROS 발행 순서의 기준이다."""
        return list(self._items)

    @property
    def item_count(self) -> int:
        """벨트가 실어 나를 수 있는 물체 수 (씬의 모든 강체)."""
        return len(self._items)

    # ── 구동 ────────────────────────────────────────────────────────────
    def _on_belt(self, name: str, x: float, y: float, z: float) -> bool:
        """이 물체가 지금 반송면에 얹혀 있는가.

        물체를 집어 올리면 z 가 이 범위를 벗어나므로 구동 대상에서 자동으로 빠진다 —
        그리퍼와 벨트가 서로 싸우지 않는 것도 이 판정 덕분이다.
        """
        rest_z = BELT_TOP_Z + self._half_height.get(name, BLOCK_HALF)
        return (
            abs(x - BELT_X) < ON_BELT_DX
            and INLET_Y - 0.06 < y < OUTLET_Y + 0.06
            and abs(z - rest_z) < REST_Z_TOL
        )

    def drive_force(self) -> int:
        """force 모드 — 벨트에 얹힌 물체에 "마찰이 끄는 힘" 을 준다.

        위치를 직접 쓰는 script 모드와 달리, 물체는 끝까지 보통 강체로 남는다.
        그래서 로봇이 집어 들거나, 서로 부딪혀 밀리거나, 기울어지는 일이 모두
        물리대로 일어난다. 벨트가 낼 수 있는 힘에 상한이 있어 무거운 물체는
        미끄러지기도 한다.

        힘은 env.step() 안의 write_data_to_sim() 에서 적용되므로 스텝 직전에
        불러야 한다.
        """
        if self.mode != "force" or not self._items:
            return 0

        origin = self.env.scene.env_origins[0]
        device = self.env.device
        driven = 0

        for name in self._items:
            obj = self.env.scene[name]
            pos = obj.data.root_pos_w[0] - origin
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])

            force = torch.zeros((1, 1, 3), device=device)
            if self.enabled and self._on_belt(name, x, y, z):
                mass = float(obj.data.default_mass[0].sum())
                vel = obj.data.root_lin_vel_w[0]
                # 벨트 진행 방향(월드 +Y)으로는 목표 속도, 폭 방향은 0 을 향한다.
                err_y = self.speed - float(vel[1])
                err_x = -float(vel[0])
                ax = max(-DRIVE_ACCEL_MAX, min(DRIVE_ACCEL_MAX, DRIVE_GAIN * err_x))
                ay = max(-DRIVE_ACCEL_MAX, min(DRIVE_ACCEL_MAX, DRIVE_GAIN * err_y))
                force[0, 0, 0] = mass * ax
                force[0, 0, 1] = mass * ay
                driven += 1

            # 벨트를 벗어난 물체는 힘을 0 으로 덮어써야 한다. 안 그러면 이전 힘이
            # 계속 남아 로봇이 들고 있는 물체를 옆으로 밀어 버린다.
            obj.set_external_force_and_torque(
                forces=force, torques=torch.zeros((1, 1, 3), device=device), is_global=True
            )

        return driven

    def _grasped(self, name: str) -> bool:
        """그리퍼가 **이** 화물에 힘을 주고 있는가.

        반드시 force_matrix_w(필터된 접촉력)를 읽어야 한다. net_forces_w 는
        센서 이름과 무관하게 **그리퍼 전체의 접촉력 합**이라, 캔 하나를 물고
        있는 동안 모든 캔의 판정이 참이 됐다 — 그래서 운반 중에는 그리퍼가
        벨트 영역을 벗어나 hold 가 풀려도 벨트 위 캔이 전부 구동에서 빠져,
        캔을 통에 놓을 때까지 컨베이어가 멈춘 것처럼 보였다(실측: 영역 이탈
        6초 뒤에도 정지, 그리퍼를 여는 순간 전진 시작).
        """
        key = self._grip_sensors.get(name)
        if key is None:
            return False
        try:
            data = self.env.scene[key].data
            fm = data.force_matrix_w
            if fm is not None:
                return float(torch.linalg.norm(fm[0])) > GRASP_CONTACT_N
            # 필터 없이 만들어진 센서라면 전체 합이라도 본다 — 없는 것보다는 낫다.
            return float(torch.linalg.norm(data.net_forces_w[0])) > GRASP_CONTACT_N
        except Exception:                                  # noqa: BLE001
            return False

    def drive(self) -> int:
        """스크립트 모드에서만 — 벨트에 얹힌 블록을 이번 스텝만큼 전진시킨다.

        표면 속도 모드에서는 PhysX 가 알아서 하므로 아무 것도 하지 않는다.
        """
        if self.mode != "script":
            return 0
        if not self.enabled or not self._items:
            return 0
        # 속도가 0 이면 쓸 것이 없다. 그런데도 위치를 다시 쓰면 **정지한 벨트가
        # 화물을 바닥에 못 박는 꼴**이 되어 로봇이 집어 올릴 수 없다.
        if self.speed == 0.0:
            return 0
        # 그리퍼가 작업 구역에 있으면 멈춘다. 회수(recycle)는 계속 돌아서
        # 출구를 지난 화물은 그대로 대기열로 들어간다.
        if self._held:
            return 0

        origin = self.env.scene.env_origins[0]
        device = self.env.device
        dt = getattr(self.env, "step_dt", None) or (1.0 / 15.0)
        advance = self.speed * dt
        driven = 0

        for name in self._items:
            obj = self.env.scene[name]
            pos = obj.data.root_pos_w[0] - origin
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            if not self._on_belt(name, x, y, z):
                continue
            # 로봇이 쥐고 있으면 손대지 않는다. 이걸 빼면 벨트가 매 스텝 화물을
            # 제자리로 되돌려서, 그리퍼가 7~10mm 들어 올려도 다음 스텝에 도로
            # 내려앉는다 — 손가락만 캔 옆면을 타고 올라가다 미끄러진 것처럼 보인다
            # (실측: 캔이 43mm 들리는 듯하다가 접촉이 끊겼다).
            if self._grasped(name):
                continue

            # 벨트 위에서는 X·Z 를 고정해 옆으로 흐르거나 가라앉지 않게 한다.
            rest_z = BELT_TOP_Z + self._half_height.get(name, BLOCK_HALF)
            pose = torch.zeros((1, 7), device=device)
            pose[0, 0] = BELT_X + origin[0]
            pose[0, 1] = y + advance + origin[1]
            pose[0, 2] = rest_z + origin[2]
            pose[0, 3:7] = obj.data.root_quat_w[0]   # 자세는 그대로 둔다
            obj.write_root_pose_to_sim(pose)

            vel = torch.zeros((1, 6), device=device)
            vel[0, 1] = self.speed
            obj.write_root_velocity_to_sim(vel)
            driven += 1

        return driven

    # ── 블록 순환 ───────────────────────────────────────────────────────
    def _positions(self) -> dict[str, torch.Tensor]:
        """순환 대상(블록)의 위치. 구동은 self._items 전체를 본다."""
        origin = self.env.scene.env_origins[0]
        return {
            name: self.env.scene[name].data.root_pos_w[0] - origin
            for name in self._blocks
        }

    def status(self) -> dict:
        """벨트 계측값 — 웹 텔레메트리와 ROS /status 로 나간다.

        others 의 값은 화물별 [y, z] 다. y 로 벨트 진행 위치를, z 로 상태
        (반송면 0.23 안팎 = 벨트 위, 음수 = 대기열, 그 사이 = 들려 있음)를 읽는다.
        """
        origin = self.env.scene.env_origins[0]
        on_belt = 0
        others = {}
        for name in self._items:
            pos = self.env.scene[name].data.root_pos_w[0] - origin
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            if self._on_belt(name, x, y, z):
                on_belt += 1
            others[name] = [round(y, 3), round(z, 3)]
        return {
            "on_belt": on_belt,
            "queued": len(self._queue),
            "others": others,
            "binned": self._binned,
            "off_belt": self._off_belt,
            "belt_held": self._held,
            "defect_ratio": round(self.defect_ratio, 2),
            "released": [self._released[False], self._released[True]],  # 정상, 불량
        }

    def recycle(self) -> int:
        """출구를 지났거나 떨어진 화물을 회수해 대기열에 넣고, 입구가 비면 투입한다.

        곧바로 입구로 되돌리지 않고 대기열을 거치는 이유는 간격 때문이다. 간격을
        넓게 잡으면 벨트에 몇 개밖에 올라가지 않는데, 그렇다고 화물 종류를 줄이면
        시연이 단조로워진다. 남는 것을 테이블 아래에 숨겨 두었다가 자리가 나면
        투입하면 종류는 그대로 두고 간격만 넓힐 수 있다.
        """
        if not self._blocks:
            return 0

        positions = self._positions()

        # 1) 회수 대상을 대기열로 보낸다.
        for name, pos in positions.items():
            if name in self._queue:
                continue
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            # 벨트 연장선상 낮은 높이면 벨트 위든 끝에서 굴러떨어졌든 회수한다.
            # X 를 좁게 보는 것이 중요하다 — 넓히면 통에 담아 둔 화물까지 끌려간다.
            past_outlet = (
                y > OUTLET_Y
                and abs(x - BELT_X) < RECOVER_DX
                and z < BELT_TOP_Z + RECOVER_DZ
            )
            # 벨트 아래로 내려간 화물 — 상판에 굴러떨어졌거나 끝에서 넘어간 것.
            # 그냥 두면 작업대에 널브러진 채 영영 남아, 로봇이 그쪽으로 팔을 뻗는다.
            # 로봇이 쥐고 있거나 통으로 옮기는 중인 것은 건드리지 않는다.
            # 상판 아래(FALLEN_Z 밑)는 이미 대기 자리에 놓인 것이라 낙하가 아니다.
            # 그걸 세면 리셋 직후마다 낙하 수가 부풀어 진단이 흐려진다.
            off_belt = (
                FALLEN_Z < z < BELT_TOP_Z - OFF_BELT_DZ
                and not self._grasped(name)
                and not self._over_bin(x, y)          # 통으로 떨어지는 중이면 놔둔다
            )
            if past_outlet or z < FALLEN_Z or self._in_bin(x, y, z) or off_belt:
                if self._in_bin(x, y, z):
                    self._binned += 1
                    why = "통"
                elif off_belt:
                    self._off_belt += 1
                    why = "벨트밖"
                elif past_outlet:
                    why = "출구통과"
                else:
                    why = "낙하"
                print(f"[recycle] {name} {why} pos=({x:.3f},{y:.3f},{z:.3f})", flush=True)
                self._park(name, len(self._queue))
                self._queue.append(name)

        # 2) 대기 중인 것은 매 스텝 붙잡아 둔다. 안 그러면 그대로 떨어진다.
        for slot, name in enumerate(self._queue):
            self._park(name, slot)

        # 3) 입구가 비었으면 하나 투입한다.
        if self._queue and self._inlet_clear(positions):
            name = self._release_next()
            if name is not None:
                self._teleport(name)
                self._released[name in self._defects] += 1
                return 1
        return 0

    def requeue_near(self, ee_pos, radius: float = 0.15) -> list[str]:
        """그리퍼 근처의 화물을 벨트에서 빼 대기열 뒤로 보낸다.

        그리퍼가 터진 자리에는 손가락을 파고든 화물이 그대로 남는다. 리셋으로
        팔만 되돌리면 그 화물이 같은 좌표에 다시 놓이고, 정책은 결정론적이라
        똑같이 접근해 똑같이 터진다 — 실측에서 같은 좌표(0.52, -0.263, 0.229)
        에서 13회 연속 재현됐다. 그래서 문제의 화물을 빼내 재투입시킨다.

        벨트가 아니라 **대기열**로 보내는 이유: 지워 버리면 순환하는 화물이
        줄어 시간이 갈수록 벨트가 비고, 그 자리에 다시 놓으면 같은 일이 반복된다.

        Returns:
            되돌린 화물 이름들.
        """
        out: list[str] = []
        ex, ey = float(ee_pos[0]), float(ee_pos[1])
        for name, pos in self._positions().items():
            if name in self._queue:
                continue
            dx, dy = float(pos[0]) - ex, float(pos[1]) - ey
            if dx * dx + dy * dy <= radius * radius:
                self._queue.append(name)
                self._park(name, len(self._queue) - 1)
                out.append(name)
        return out

    def _release_next(self) -> str | None:
        """대기열에서 하나 골라 뺀다. 불량품 비율을 **쿼터**로 지킨다.

        투입마다 독립 확률(동전던지기)로 정하면 안 된다 — 정상만 줄줄이 나오는
        구간이 얼마든지 길어진다. 실측: seed 0 은 난수가 0.2 아래로 처음 떨어지는
        것이 26번째라, 불량 20% 환경인데 정상 캔만 25번 투입되는 동안 불량품이
        한 번도 벨트에 오르지 않았다. 그래서 지금까지 실제 투입된 불량 비율이
        목표에 못 미칠 때 불량을 낸다. 20% 면 다섯 번째마다 하나 — 장기 평균이
        정확히 맞고, 첫 불량이 언제 나올지도 예측된다(평가 환경엔 이쪽이 낫다).

        원하는 종류가 대기열에 없으면 그냥 맨 앞을 낸다 — 비율을 맞추겠다고
        벨트를 비워 두면 조작할 것이 없어진다. 투입 카운터(_released)는 실제로
        나간 것 기준으로 세므로(recycle 참고) 그런 대체 투입도 비율에 반영된다.
        """
        if not self._queue:
            return None
        total = self._released[False] + self._released[True]
        # 올림 방향 쿼터 — 20% 면 D N N N N D … 로 **첫 투입이 불량**이다.
        # 내림 방향(N N N N D …)으로 했더니 벨트 시작 캔 3개(정상)까지 겹쳐
        # 기동 후 1분 넘게 불량이 안 보였고, 불량 20% 환경을 켠 사람이 불량을
        # 구경도 못 하는 채로 지나갔다. 장기 비율은 어느 쪽이든 같다.
        want_defect = self._released[True] < (total + 1) * self.defect_ratio
        for i, name in enumerate(self._queue):
            if (name in self._defects) == want_defect:
                return self._queue.pop(i)
        return self._queue.pop(0)

    def _inlet_clear(self, positions: dict) -> bool:
        """입구 근처가 비어 있는가. 대기 중인 화물은 세지 않는다."""
        for name, pos in positions.items():
            if name in self._queue:
                continue
            if (
                abs(float(pos[1]) - INLET_Y) < self.spacing
                and abs(float(pos[0]) - BELT_X) < 0.12
            ):
                return False
        return True

    def _park(self, name: str, slot: int = 0) -> None:
        """대기 자리(테이블 아래)에 붙잡아 둔다.

        여러 개가 한 점에 겹치면 물리 솔버가 불필요하게 밀어내므로 슬롯마다
        아래로 조금씩 띄운다. 어차피 상판에 가려 보이지 않는다.
        """
        obj = self.env.scene[name]
        device = self.env.device
        origin = self.env.scene.env_origins[0]

        pose = torch.zeros((1, 7), device=device)
        pose[0, 0] = STAGING_POS[0] + origin[0]
        pose[0, 1] = STAGING_POS[1] + origin[1]
        pose[0, 2] = STAGING_POS[2] - 0.12 * slot + origin[2]
        pose[0, 3] = 1.0
        obj.write_root_pose_to_sim(pose)
        obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=device))

    def inlet_y(self) -> float:
        """새 화물을 놓을 y. 벨트가 멈춰 있으면 조금 안쪽에 놓는다.

        입구(-0.36)는 벨트 끝(-0.40)에서 4cm 밖에 안 떨어져 있다. 벨트가 흐르면
        곧바로 안쪽으로 이동하지만, 멈춰 있으면 그 자리에 그대로 놓여 뒤로
        굴러떨어지는 일이 있었다.
        """
        return INLET_Y if not self._held else INLET_Y + 0.06

    def _teleport(self, name: str) -> None:
        """화물을 입구로 옮기고 속도를 0으로 만든다.

        속도를 지우지 않으면 이전 낙하 속도가 남아 벨트를 뚫고 내려간다.
        높이는 화물별 반높이로 계산한다 — 블록 기준 상수를 쓰면 키가 큰 캔이
        반송면에 파묻힌 채로 투입된다.
        """
        obj = self.env.scene[name]
        device = self.env.device
        origin = self.env.scene.env_origins[0]

        pose = torch.zeros((1, 7), device=device)
        pose[0, 0] = BELT_X + origin[0]
        pose[0, 1] = self.inlet_y() + origin[1]
        pose[0, 2] = (
            BELT_TOP_Z + self._half_height.get(name, BLOCK_HALF) + 0.002 + origin[2]
        )
        pose[0, 3] = 1.0  # quat w — 자세도 같이 초기화한다

        obj.write_root_pose_to_sim(pose)
        obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=device))
