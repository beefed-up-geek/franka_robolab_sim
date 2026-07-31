# SPDX-License-Identifier: Apache-2.0
"""EEF 목표 안전 클램프.

상대 IK는 델타를 그대로 적분하므로, 사용자가 키를 계속 누르면 EEF가 작업공간
밖으로 밀려나 IK가 발산한다. 매 스텝 "현재 위치 + 델타"를 안전박스와 반경으로
자른 뒤 다시 델타로 되돌려 발산을 막는다.

실물 FR3 쪽 teleop_franka.py 의 BOX_LO/BOX_HI/R_MAX 가드와 같은 개념이다.
"""
from __future__ import annotations

import torch

from franka_teleop import config


def clamp_delta(ee_pos: torch.Tensor, delta_pos: torch.Tensor) -> torch.Tensor:
    """목표(현재+델타)를 안전영역 안으로 자른 뒤 허용 델타를 돌려준다.

    Args:
        ee_pos:    (3,) 로봇 베이스 기준 현재 EEF 위치 [m]
        delta_pos: (3,) 이번 스텝에 적용하려는 위치 델타 [m]

    Returns:
        (3,) 클램프된 위치 델타.
    """
    lo = torch.tensor(config.SAFE_BOX_LO, device=delta_pos.device, dtype=delta_pos.dtype)
    hi = torch.tensor(config.SAFE_BOX_HI, device=delta_pos.device, dtype=delta_pos.dtype)

    target = ee_pos + delta_pos
    target = torch.max(torch.min(target, hi), lo)

    # 반경 제한 — 박스 모서리는 팔이 닿지 않는 영역이라 구로 한 번 더 자른다.
    r = torch.linalg.norm(target)
    if r > config.SAFE_RADIUS:
        target = target * (config.SAFE_RADIUS / r)

    return target - ee_pos


def is_outside(ee_pos: torch.Tensor) -> bool:
    """현재 EEF가 이미 안전영역을 벗어났는지 — UI 경고 표시용."""
    lo = torch.tensor(config.SAFE_BOX_LO, device=ee_pos.device, dtype=ee_pos.dtype)
    hi = torch.tensor(config.SAFE_BOX_HI, device=ee_pos.device, dtype=ee_pos.dtype)
    if bool((ee_pos < lo).any() or (ee_pos > hi).any()):
        return True
    return bool(torch.linalg.norm(ee_pos) > config.SAFE_RADIUS)
