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

# 벨트 콜라이더 프림 이름 (assets/conveyor/conveyor.usda)
BELT_PRIM_NAME = "belt_surface"

# 씬 배치 기준 — franka_teleop/world_assets.py 의 컨베이어 배치와 맞물린다.
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
REST_Z = BELT_TOP_Z + BLOCK_HALF     # 블록 기준 (회수 위치 계산용)
REST_Z_TOL = 0.015                   # 이보다 들리면 구동 대상에서 제외
ON_BELT_DX = 0.09                    # 벨트 폭 방향 허용치

# 입구에 블록이 겹쳐 쌓이지 않도록, 이 거리 안에 다른 블록이 있으면 투입을 미룬다.
INLET_CLEARANCE = 0.11

# 테이블 아래로 완전히 떨어진 물체는 어디에 있든 회수한다.
FALLEN_Z = -0.10

# 출구 너머 회수 판정 범위. X 는 벨트 연장선에서 크게 벗어나지 않아야 하고
# (그릇에 담긴 블록을 건드리지 않으려면 좁아야 한다), Z 는 로봇이 들어 올린
# 상태를 걸러내는 용도다.
RECOVER_DX = 0.13
RECOVER_DZ = 0.08

# 벨트 위에서 옆으로 흐르거나 구르는 것을 줄이는 감쇠 (script 모드 전용)
LATERAL_DAMPING = 0.3
ANGULAR_DAMPING = 0.4

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

    def __init__(self, env, speed: float, mode: str = "surface") -> None:
        self.env = env
        self.speed = speed
        self.enabled = True
        self.mode = mode
        self._surface_apis: list = []

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
        self._belt_found = self._check_belt()
        if self.mode == "surface":
            self._apply_surface_velocity()

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
            api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
            api.CreateSurfaceVelocityEnabledAttr(True)
            # 로컬 좌표로 줘야 컨베이어를 씬에서 어떻게 돌려 놓든 로컬 +X 가
            # 진행 방향이 된다.
            api.CreateSurfaceVelocityLocalSpaceAttr(True)
            api.CreateSurfaceVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            self._surface_apis.append(api)
        self._push_surface_speed()

    def _push_surface_speed(self) -> None:
        from pxr import Gf

        value = self.speed if self.enabled else 0.0
        for api in self._surface_apis:
            api.GetSurfaceVelocityAttr().Set(Gf.Vec3f(value, 0.0, 0.0))

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

    def drive(self) -> int:
        """스크립트 모드에서만 — 벨트에 얹힌 블록을 이번 스텝만큼 전진시킨다.

        표면 속도 모드에서는 PhysX 가 알아서 하므로 아무 것도 하지 않는다.
        """
        if self.mode != "script":
            return 0
        if not self.enabled or not self._items:
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
        """블록이 실제로 벨트를 타고 있는지 보기 위한 계측값.

        block_vy 는 명령한 속도가 실제로 유지되는지 확인용이다 — 마찰에 제동되면
        여기서 바로 드러난다.
        """
        origin = self.env.scene.env_origins[0]
        on_belt, ys, zs, vys = 0, [], [], []
        for name in (n for n in self._blocks if n.startswith("block_")):
            obj = self.env.scene[name]
            pos = obj.data.root_pos_w[0] - origin
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            ys.append(round(y, 3))
            zs.append(round(z, 3))
            vys.append(round(float(obj.data.root_vel_w[0, 1]), 4))
            if self._on_belt(name, x, y, z):
                on_belt += 1
        # 블록이 아닌 물체도 실려 가는지 확인하기 위한 계측.
        origin2 = self.env.scene.env_origins[0]
        others = {}
        items_on_belt = 0
        for name in self._items:
            pos = self.env.scene[name].data.root_pos_w[0] - origin2
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            if self._on_belt(name, x, y, z):
                items_on_belt += 1
            if not name.startswith("block_"):
                others[name] = [round(y, 3), round(z, 3)]
        return {
            "on_belt": on_belt,
            "block_y": ys,
            "block_z": zs,
            "block_vy": vys,
            "items_on_belt": items_on_belt,
            "others": others,
        }

    def recycle(self) -> int:
        """출구를 지났거나 떨어진 블록을 입구로 되돌린다. 되돌린 개수를 반환."""
        if not self._blocks:
            return 0

        positions = self._positions()
        moved = 0

        for name, pos in positions.items():
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            # 출구를 지난 화물을 회수한다. "벨트에 얹혀 있을 때" 로만 좁히면
            # 둥근 물체가 끝에서 굴러떨어졌을 때 테이블에 그대로 방치된다
            # (원기둥으로 확인). 그래서 벨트 연장선상에서 낮은 높이에 있으면
            # 벨트 위든 떨어졌든 모두 회수 대상으로 본다.
            #
            # X 범위를 좁게 잡는 것이 중요하다. 넓히면 그릇에 담아 둔 블록까지
            # 출구 너머로 판정되어 입구로 끌려가 버린다.
            past_outlet = (
                y > OUTLET_Y
                and abs(x - BELT_X) < RECOVER_DX
                and z < BELT_TOP_Z + RECOVER_DZ   # 로봇이 들어 올린 중이면 제외
            )
            fallen = z < FALLEN_Z
            if not (past_outlet or fallen):
                continue

            # 입구가 비어 있을 때만 투입한다. 아니면 다음 스텝에 다시 시도.
            crowded = any(
                other != name
                and abs(float(p[1]) - INLET_Y) < INLET_CLEARANCE
                and abs(float(p[0]) - BELT_X) < 0.12
                for other, p in positions.items()
            )
            if crowded and not fallen:
                continue

            self._teleport(name)
            positions[name] = torch.tensor([BELT_X, INLET_Y, REST_Z], device=pos.device)
            moved += 1

        return moved

    def _teleport(self, name: str) -> None:
        """블록을 입구로 옮기고 속도를 0으로 만든다.

        속도를 지우지 않으면 이전 낙하 속도가 남아 벨트를 뚫고 내려간다.
        """
        obj = self.env.scene[name]
        device = self.env.device
        origin = self.env.scene.env_origins[0]

        pose = torch.zeros((1, 7), device=device)
        pose[0, 0] = BELT_X + origin[0]
        pose[0, 1] = INLET_Y + origin[1]
        pose[0, 2] = REST_Z + 0.002 + origin[2]
        pose[0, 3] = 1.0  # quat w — 자세도 같이 초기화한다

        obj.write_root_pose_to_sim(pose)
        obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=device))
