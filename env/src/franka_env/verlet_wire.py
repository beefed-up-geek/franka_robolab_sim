# SPDX-License-Identifier: Apache-2.0
"""task2 전선 — 게임식 Verlet 로프 (시각 전용, 양끝 핀 고정).

PhysX 밖에서 입자 사슬(당김 전용 거리 제약 + 중력 + 감쇠)을 적분하고,
그 곡선으로 USD 튜브 메시(고정 토폴로지)의 points 만 매 스텝 다시 쓴다.
물리와 완전히 분리되어 있어 파지·삽입·리셋과 간섭하지 않는다 — 관절
로프에서 실측된 리셋 폭발·얽힘·끌림이 구조적으로 불가능하다.
(Jakobsen "Advanced Character Physics" 계열, Half-Life 2 식 케이블)

한쪽 끝(입자 0·1)은 발전기 DC 소켓 앞의 고정점, 반대쪽 끝(입자 -1·-2)은
커넥터의 케이블 글랜드 입구에 매 스텝 핀 고정한다 — 커넥터를 들면
전선이 따라온다. 베이스 곡선·rest 길이는 물리 정착 결과에서 구웠다
(asset/objects/task2/wire_curves.json, tools/settle_wire.py 참고).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from pxr import Vt

_CURVES = Path(__file__).resolve().parents[2] / "asset" / "objects" / "task2" / "wire_curves.json"
_PAIR = {"wire_red": "connector_red", "wire_black": "connector_black"}
_GLAND_TIP = np.array([0.034, 0.0, 0.06])   # 커넥터 로컬: 글랜드 입구
_DT = 1.0 / 60.0
_G = 9.81
_DAMP = 0.985
_ITERS = 12
_ZMIN = 0.0065        # 튜브 반지름 + 여유 — 상판 관통 방지
_WARMUP = 150         # 리셋 직후 정착 반복 수
_WRITE_EVERY = 2      # points 갱신 주기 (적분은 매 스텝)


def _quat_rot(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    return np.array([
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - w * z) * vy + 2 * (x * z + w * y) * vz,
        2 * (x * y + w * z) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - w * x) * vz,
        2 * (x * z - w * y) * vx + 2 * (y * z + w * x) * vy + (1 - 2 * (x * x + y * y)) * vz,
    ])


def _tube_points(centers, radius, sides):
    """평행 이동 프레임 스윕 — 씬 생성기(tools)와 같은 토폴로지 (P*S+2)."""
    c = centers
    n = len(c)
    tang = np.empty_like(c)
    tang[0] = c[1] - c[0]
    tang[-1] = c[-1] - c[-2]
    tang[1:-1] = c[2:] - c[:-2]
    tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-9)
    N = np.empty_like(c)
    B = np.empty_like(c)
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(tang[0], ref))) > 0.9:
        ref = np.array([1.0, 0.0, 0.0])
    N[0] = np.cross(tang[0], ref)
    N[0] /= max(float(np.linalg.norm(N[0])), 1e-9)
    for i in range(1, n):
        v = np.cross(tang[i - 1], tang[i])
        s = float(np.linalg.norm(v))
        if s < 1e-9:
            N[i] = N[i - 1]
        else:
            k = v / s
            th = math.atan2(s, float(np.clip(np.dot(tang[i - 1], tang[i]), -1, 1)))
            nv = N[i - 1]
            N[i] = (nv * math.cos(th) + np.cross(k, nv) * math.sin(th)
                    + k * float(np.dot(k, nv)) * (1.0 - math.cos(th)))
        N[i] -= tang[i] * float(np.dot(N[i], tang[i]))
        N[i] /= max(float(np.linalg.norm(N[i])), 1e-9)
    B = np.cross(tang, N)
    ang = np.linspace(0.0, 2.0 * math.pi, sides, endpoint=False)
    rings = (c[:, None, :]
             + radius * (np.cos(ang)[None, :, None] * N[:, None, :]
                         + np.sin(ang)[None, :, None] * B[:, None, :]))
    return np.concatenate([rings.reshape(-1, 3), c[:1], c[-1:]],
                          axis=0).astype(np.float32)


class _WireSim:
    def __init__(self, attr, spec):
        self.attr = attr
        self.base = np.asarray(spec["base"], dtype=np.float64)
        self.rest = np.asarray(spec["rest"], dtype=np.float64)
        self.radius = float(spec["radius"])
        self.sides = int(spec["sides"])
        # 베이스 곡선은 씬에 구워진 튜브의 링 중심에서 복원한다 — 씬(train/
        # test)마다 휴지 배치가 달라도 json 은 rest(로프 길이)만 공유하면 된다.
        raw = attr.Get()
        n, s = len(self.base), self.sides
        if raw is not None and len(raw) == n * s + 2:
            self.base = np.asarray(raw, dtype=np.float64)[: n * s] \
                .reshape(n, s, 3).mean(axis=1)
        self.p = self.base.copy()
        self.prev = self.base.copy()
        self.pending = True     # 첫 스텝에 정착(warmup)부터
        self.tick = 0

    def _pin(self, gpos, gquat):
        self.p[0] = self.base[0]
        self.p[1] = self.base[1]
        tip = gpos + _quat_rot(gquat, _GLAND_TIP)
        gdir = _quat_rot(gquat, np.array([1.0, 0.0, 0.0]))
        self.p[-1] = tip
        self.p[-2] = tip + gdir * self.rest[-1]

    def _integrate(self, gpos, gquat):
        p, prev = self.p, self.prev
        vel = (p - prev) * _DAMP
        prev[:] = p
        p += vel
        p[2:-2, 2] -= _G * _DT * _DT
        for _ in range(_ITERS):
            self._pin(gpos, gquat)
            d = p[1:] - p[:-1]
            dist = np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
            excess = np.maximum(dist - self.rest[:, None], 0.0)
            corr = d / dist * excess * 0.5
            p[:-1] += corr
            p[1:] -= corr
            np.maximum(p[2:-2, 2], _ZMIN, out=p[2:-2, 2])
        self._pin(gpos, gquat)

    def step(self, gpos, gquat):
        gpos = np.asarray(gpos, dtype=np.float64)
        if self.pending:
            self.pending = False
            self.p[:] = self.base
            self.prev[:] = self.base
            for _ in range(_WARMUP):
                self._integrate(gpos, gquat)
            self.prev[:] = self.p
        else:
            self._integrate(gpos, gquat)
        self.tick += 1
        if self.tick % _WRITE_EVERY:
            return
        # 렌더용 한 번의 라플라시안 스무딩 — 꺾임을 눅인다 (입자 수 불변)
        pr = self.p.copy()
        pr[1:-1] = 0.25 * self.p[:-2] + 0.5 * self.p[1:-1] + 0.25 * self.p[2:]
        pts = _tube_points(pr, self.radius, self.sides)
        try:
            arr = Vt.Vec3fArray.FromNumpy(pts)
        except AttributeError:
            arr = Vt.Vec3fArray([tuple(v) for v in pts])
        self.attr.Set(arr)


class Task2Wires:
    """씬의 wire_red/wire_black 튜브 메시를 Verlet 로프로 구동한다."""

    def __init__(self, stage):
        spec = json.load(open(_CURVES))
        prims = {}
        for prim in stage.Traverse():
            if prim.GetName() in _PAIR and prim.GetTypeName() == "Mesh":
                prims[prim.GetName()] = prim
        self.sims = {}
        for wname, cname in _PAIR.items():
            if wname in prims and wname in spec:
                attr = prims[wname].GetAttribute("points")
                self.sims[cname] = _WireSim(attr, spec[wname])
        self.ok = len(self.sims) == 2

    def step(self, connector, pos, quat):
        sim = self.sims.get(connector)
        if sim is not None:
            sim.step(pos, quat)

    def request_reset(self):
        for sim in self.sims.values():
            sim.pending = True
