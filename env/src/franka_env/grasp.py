# SPDX-License-Identifier: Apache-2.0
"""물체를 집으려면 그리퍼를 어디에 둬야 하는지 계산한다.

## 왜 상수를 박지 않는가

`/franka/eef_pose` 가 내보내는 자세는 **손끝이 아니라 Robotiq 2F-85 장착 플랜지**
(`base_link`)다. 그래서 "캔 중심에 손끝을 두려면 플랜지를 어디에 둬야 하는가" 를
알아야 하는데, 그 오프셋을 상수로 적으면 그리퍼를 바꾸는 순간 조용히 틀린다.
RoboLab 소스에 주석 처리된 `body_offset pos=[0,0,0.1628]` 이 있지만 주석인 데는
이유가 있어 보이고, 홈 자세에서 실제로 바닥을 향하는 축은 로컬 +Z 가 아니라
**로컬 +X** 였다(실측). 그래서 손가락 링크 위치를 로봇에서 직접 재서 쓴다.

## 어떤 자세로 잡는가

캔은 원기둥이라 위에서 수직으로 내려 잡는 것이 자연스럽고, 옆에서 잡으면 컨베이어
난간과 부딪힌다. 방위(yaw)는 원기둥이라 아무 값이나 되므로 홈 자세의 방향을 그대로
쓴다 — 로봇이 이미 아래를 보고 있어서 추가 회전이 필요 없다.

높이는 **물체 중심**이다. 뚜껑 쪽을 잡으면 파열 캔의 부푼 돔에 미끄러지고, 바닥
쪽을 잡으면 벨트와 부딪힌다.
"""
from __future__ import annotations

import torch


class GraspSolver:
    """플랜지 ↔ 손끝 오프셋을 로봇에서 재고, 물체별 파지 자세를 만든다."""

    def __init__(self, env, ee_pos, ee_quat) -> None:
        self.ok = False
        self.offset_local = None      # 플랜지 좌표계에서 본 손끝 위치
        self.home_quat = None         # 위에서 내려 잡는 기준 자세
        # 손가락 **링크 원점** 은 쓸 수 없다. USD 에서 관절 프레임이 부모 쪽에
        # 놓여 있어서 네 손가락 링크가 전부 플랜지와 같은 자리로 나온다(실측:
        # 오프셋이 0,0,0). 그래서 손가락 지오메트리의 실제 아랫끝을 잰다.
        try:
            import omni.usd
            from pxr import Usd, UsdGeom
        except ImportError:
            print("[grasp] USD 를 쓸 수 없어 파지 자세를 끕니다.", flush=True)
            return

        stage = omni.usd.get_context().get_stage()
        lo_z, names = None, []
        for prim in stage.Traverse():
            n = prim.GetName().lower()
            if "finger" not in n and "pad" not in n:
                continue
            rng = (
                UsdGeom.Imageable(prim)
                .ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
                .ComputeAlignedRange()
            )
            if rng.IsEmpty():
                continue
            z = float(rng.GetMin()[2])
            names.append(prim.GetName())
            lo_z = z if lo_z is None else min(lo_z, z)

        if lo_z is None:
            print("[grasp] 손가락 지오메트리를 찾지 못해 파지 자세를 끕니다.", flush=True)
            return

        flange = ee_pos[0].detach().cpu()
        quat = ee_quat[0].detach().cpu()
        # 홈 자세에서 그리퍼는 수직 아래를 보므로 손끝은 플랜지 **바로 아래**다.
        tcp = torch.tensor([float(flange[0]), float(flange[1]), lo_z], dtype=flange.dtype)
        self.offset_local = _rotate(_conj(quat), tcp - flange)
        self.home_quat = quat.clone()
        self.ok = True
        print(
            f"[grasp] 손끝 z={lo_z:.4f} (프림 {len(names)}개) → 플랜지에서 "
            f"{[round(float(v), 4) for v in self.offset_local.tolist()]} m (로컬), "
            f"수직 거리 {float(flange[2]) - lo_z:.4f} m",
            flush=True,
        )

    def flange_for(self, target_w: torch.Tensor) -> torch.Tensor:
        """손끝을 target_w 에 두려면 플랜지가 있어야 할 월드 위치."""
        return target_w - _rotate(self.home_quat.to(target_w.device), self.offset_local.to(target_w.device))


def _conj(q: torch.Tensor) -> torch.Tensor:
    return torch.stack([q[0], -q[1], -q[2], -q[3]])


def _rotate(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """쿼터니언 (w,x,y,z) 으로 벡터를 돌린다."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    u = torch.stack([x, y, z])
    return v + 2.0 * torch.cross(u, torch.cross(u, v, dim=-1) + w * v, dim=-1)
