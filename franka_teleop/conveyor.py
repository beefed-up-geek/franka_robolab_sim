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

# 블록이 벨트에 "얹혀 있다" 고 볼 범위.
REST_Z = BELT_TOP_Z + BLOCK_HALF     # 얹혔을 때의 중심 높이
REST_Z_TOL = 0.012                   # 이보다 들리면 구동 대상에서 제외
ON_BELT_DX = 0.09                    # 벨트 폭 방향 허용치

# 입구에 블록이 겹쳐 쌓이지 않도록, 이 거리 안에 다른 블록이 있으면 투입을 미룬다.
INLET_CLEARANCE = 0.11

# 테이블 아래로 완전히 떨어진 블록만 회수한다. 상판 근처를 기준으로 삼으면
# 그릇에 담아 둔 블록까지 입구로 끌려간다.
FALLEN_Z = -0.10

# 벨트 위에서 옆으로 흐르거나 구르는 것을 줄이는 감쇠 (1.0 이면 감쇠 없음)
LATERAL_DAMPING = 0.3
ANGULAR_DAMPING = 0.4


class Conveyor:
    """벨트 위 블록을 구동하고, 출구를 지난 블록을 입구로 되돌린다."""

    def __init__(self, env, speed: float) -> None:
        self.env = env
        self.speed = speed
        self.enabled = True

        # env.scene 에는 블록 강체뿐 아니라 접촉 센서도 들어 있고, 센서 이름이
        # "block_0__block_1" 처럼 같은 접두사로 시작한다. 이름만 보고 고르면
        # 센서가 섞여 들어와 root_pos_w 를 읽다 죽는다. 타입으로 걸러야 한다.
        from isaaclab.assets import RigidObject

        self._blocks: list[str] = sorted(
            name
            for name in env.scene.keys()
            if name.startswith("block_") and isinstance(env.scene[name], RigidObject)
        )
        self._belt_found = self._check_belt()

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

    def set_speed(self, speed: float) -> None:
        self.speed = speed

    @property
    def ready(self) -> bool:
        return self._belt_found and bool(self._blocks)

    @property
    def block_count(self) -> int:
        return len(self._blocks)

    # ── 구동 ────────────────────────────────────────────────────────────
    def _on_belt(self, x: float, y: float, z: float) -> bool:
        return (
            abs(x - BELT_X) < ON_BELT_DX
            and INLET_Y - 0.06 < y < OUTLET_Y + 0.06
            and abs(z - REST_Z) < REST_Z_TOL
        )

    def drive(self) -> int:
        """벨트에 얹힌 블록을 이번 스텝만큼 전진시킨다. 구동한 개수를 반환."""
        if not self.enabled or not self._blocks:
            return 0

        origin = self.env.scene.env_origins[0]
        device = self.env.device
        dt = getattr(self.env, "step_dt", None) or (1.0 / 15.0)
        advance = self.speed * dt
        driven = 0

        for name in self._blocks:
            obj = self.env.scene[name]
            pos = obj.data.root_pos_w[0] - origin
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            if not self._on_belt(x, y, z):
                continue

            # 벨트 위에서는 X·Z 를 고정해 옆으로 흐르거나 가라앉지 않게 한다.
            pose = torch.zeros((1, 7), device=device)
            pose[0, 0] = BELT_X + origin[0]
            pose[0, 1] = y + advance + origin[1]
            pose[0, 2] = REST_Z + origin[2]
            pose[0, 3:7] = obj.data.root_quat_w[0]   # 자세는 그대로 둔다
            obj.write_root_pose_to_sim(pose)

            vel = torch.zeros((1, 6), device=device)
            vel[0, 1] = self.speed
            obj.write_root_velocity_to_sim(vel)
            driven += 1

        return driven

    # ── 블록 순환 ───────────────────────────────────────────────────────
    def _positions(self) -> dict[str, torch.Tensor]:
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
        for name in self._blocks:
            obj = self.env.scene[name]
            pos = obj.data.root_pos_w[0] - origin
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            ys.append(round(y, 3))
            zs.append(round(z, 3))
            vys.append(round(float(obj.data.root_vel_w[0, 1]), 4))
            if self._on_belt(x, y, z):
                on_belt += 1
        return {"on_belt": on_belt, "block_y": ys, "block_z": zs, "block_vy": vys}

    def recycle(self) -> int:
        """출구를 지났거나 떨어진 블록을 입구로 되돌린다. 되돌린 개수를 반환."""
        if not self._blocks:
            return 0

        positions = self._positions()
        moved = 0

        for name, pos in positions.items():
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            # 반드시 벨트 위에 있을 때만 출구로 친다. 로봇이 블록을 집어 출구
            # 너머로 옮기는 중에 회수되면 그리퍼에서 블록이 사라진다.
            past_outlet = self._on_belt(x, y, z) and y > OUTLET_Y
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
