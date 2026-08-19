#!/usr/bin/env python3
"""VLA_lang — 수집 데이터에 **steering command** 를 합성해 재적재한다.

Steerable Policies (Chen et al., arXiv:2602.13193 — methods/language-controllable-vla)
방식이다. BC 학습의 태스크 문장을 프레임마다 여러 추상화 수준의 명령으로 무작위
치환해, 태스크 문장뿐 아니라 **부분목표·원자 동작·좌표 지시**까지 따르는 정책을
만든다. 논문은 실로봇 데이터라 Molmo/SAM2/Gemini 로 명령을 뽑았지만, 우리는
시뮬레이터라 수집기가 남긴 단계(stage)·목표(target)·목표 좌표(target_*)가 곧
정답 주석이다 — 합성 파이프라인이 통째로 공짜다.

명령 4계층 (프레임마다 균등 무작위로 하나):
    task     원래 태스크 문장 ("Pick up the cans ...")
    subtask  단계에서 유도 ("reach for the can", "grasp the hammer", ...)
    motion   앞으로 몇 프레임의 액션 방향에서 유도 ("move left", "move down", ...)
    point    **좌표 지시** ("reach for the can at [0.52, -0.06]", "go to [0.26, 0.58]")

좌표는 로봇 베이스(월드) 좌표 [m, 소수 2자리]다. 논문은 픽셀을 쓰지만 우리
정책의 상태 공간이 월드 EEF 좌표라 같은 좌표계가 자연스럽고, 추론 때 사람이
지시를 만들기도 쉽다. 물체 좌표의 출처는 두 가지다:
    task3 v10   수집기가 프레임마다 남긴 target_x/y (벨트가 저속으로 흘러 상수가 아님)
    task1/2     파지 순간(CLOSE 진입)의 EEF xy — 물체가 리셋 자세에 고정이라 정확하다

action 은 abs([x y z gripper])만 만든다 — 세 태스크 모두 abs 가 delta 를 이겼다.

사용:
    python ingest_lang.py --src <v2.0 데이터셋> --dst <출력> --task task1|task2|task3
"""
from __future__ import annotations

import argparse
import json
import random
import shutil
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq

from lerobot.datasets.lerobot_dataset import LeRobotDataset

CAMS = ("front", "top", "wrist")

# 태스크 문장 (기존 학습과 동일 — inference/run_policy.py TASK_TEXT 와 맞춘다)
TASK_TEXT = {
    "task1": "Hand the tool to the worker",
    "task2": "Plug the red charging connector into the battery positive terminal",
    "task3": "Pick up the cans from the conveyor and put them in the bin",
}

# 대상 이름 → 문장 속 명사구
NOUN = {
    "hammer_7": "the hammer", "cordless_drill": "the drill",
    "connector_red": "the red connector", "connector_black": "the black connector",
}

# 고정 좌표 [m] — 씬 배치 상수 (data_collection/*/policy.py 와 동일 출처)
BIN_XY = (0.26, 0.58)        # task3 담는 통 (task3/policy.py BIN_XY)
HOME_XY = (0.36, 0.00)       # task3 홈 복귀 (policy.py HOME_POS)
CROSS_XY = (0.48, -0.58)     # task1 전달 지점 (task1/policy.py CROSS_XY)

# 원자 동작 — 로봇 베이스 기준 축 이름. +x 앞, +y 왼쪽, +z 위.
AXIS_WORDS = {
    (0, +1): "move forward", (0, -1): "move backward",
    (1, +1): "move left",    (1, -1): "move right",
    (2, +1): "move up",      (2, -1): "move down",
}
MOTION_WINDOW = 5        # 방향은 앞 5프레임 액션 평균으로 정한다 (한 프레임은 잡음)
MOTION_MIN = 0.004       # 지배축 평균이 이보다 작으면 동작 명령을 만들지 않는다

STYLES = ("task", "subtask", "motion", "point")


def can_noun(target: str) -> str:
    return NOUN.get(target, "the can")


def fmt(x: float, y: float) -> str:
    return f"[{x:.2f}, {y:.2f}]"


# ── 단계 → subtask / point 문장 ──────────────────────────────────────────
def subtask_of(task: str, stage: str, target: str) -> str | None:
    n = can_noun(target)
    if task == "task3":
        return {
            "VIA": f"reach for {n}", "TRANSIT": f"reach for {n}",
            "APPROACH": f"reach for {n}", "DESCEND": f"move down to {n}",
            "CLOSE": f"grasp {n}", "LIFT": f"lift {n}",
            "TO_BIN": f"carry {n} to the bin",
            "OPEN": f"drop {n} into the bin",
            "HOME": "return to the home position",
        }.get(stage)
    if task == "task1":
        return {
            "TRANSIT": f"reach for {n}", "APPROACH": f"reach for {n}",
            "DESCEND": f"move down to {n}", "CLOSE": f"grasp {n}",
            "LIFT": f"lift {n}", "DELIVER": f"carry {n} to the worker",
        }.get(stage)
    if task == "task2":
        return {
            "TRANSIT": f"reach for {n}", "APPROACH": f"reach for {n}",
            "DESCEND": f"move down to {n}", "CLOSE": f"grasp {n}",
            "LIFT": f"lift {n}", "MOVE": "move to the battery terminal",
            "ALIGN": "align the connector with the terminal",
            "PLACE": "plug the connector into the terminal",
        }.get(stage)
    return None


def point_of(task: str, stage: str, target: str,
             obj_xy, goal_xy) -> str | None:
    """좌표 지시. obj_xy 는 접근 대상(캔·공구·커넥터), goal_xy 는 놓는 곳."""
    n = can_noun(target)
    reach = ("VIA", "TRANSIT", "APPROACH", "DESCEND")
    if stage in reach and obj_xy is not None:
        return f"reach for {n} at {fmt(*obj_xy)}"
    if stage == "CLOSE" and obj_xy is not None:
        return f"grasp {n} at {fmt(*obj_xy)}"
    if task == "task3":
        if stage in ("TO_BIN", "OPEN"):
            return f"put {n} in the bin at {fmt(*BIN_XY)}"
        if stage == "HOME":
            return f"move back to {fmt(*HOME_XY)}"
    if task == "task1" and stage == "DELIVER":
        return f"carry {n} to {fmt(*CROSS_XY)}"
    if task == "task2" and stage in ("MOVE", "ALIGN", "PLACE") and goal_xy is not None:
        return f"plug {n} into the terminal at {fmt(*goal_xy)}"
    return None


def motion_of(action: np.ndarray, i: int) -> str | None:
    """앞 MOTION_WINDOW 프레임의 평균 이동 방향에서 원자 동작 문장을 만든다.

    그리퍼 전이가 그 창 안에 있으면 이동보다 그쪽이 그 순간의 '동작'이다.
    """
    w = action[i:i + MOTION_WINDOW]
    if len(w) >= 2:
        g0, g1 = w[0, 3], w[-1, 3]
        if g0 < 0.5 <= g1:
            return "close the gripper"
        if g1 < 0.5 <= g0:
            return "open the gripper"
    mean = w[:, :3].mean(axis=0)
    ax = int(np.argmax(np.abs(mean)))
    if abs(mean[ax]) < MOTION_MIN:
        return None
    return AXIS_WORDS[(ax, +1 if mean[ax] > 0 else -1)]


# ── 에피소드 적재 ────────────────────────────────────────────────────────
def read_episode(src: Path, idx: int):
    t = pq.read_table(src / "data" / "chunk-000" / f"episode_{idx:06d}.parquet")
    cols = t.column_names
    d = {
        "state": np.array(t.column("observation.state").to_pylist(), np.float32),
        "action": np.array(t.column("action").to_pylist(), np.float32),
        "stage": t.column("stage").to_pylist() if "stage" in cols else None,
        "target": t.column("target").to_pylist() if "target" in cols else None,
    }
    if "target_x" in cols:
        d["txy"] = np.array([t.column("target_x").to_pylist(),
                             t.column("target_y").to_pylist()], np.float64).T
    else:
        d["txy"] = None
    videos = {}
    for cam in CAMS:
        cap = cv2.VideoCapture(str(src / "videos" / "chunk-000"
                                   / f"observation.images.{cam}" / f"episode_{idx:06d}.mp4"))
        frames = []
        while True:
            ok, f = cap.read()
            if not ok:
                break
            frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))
        cap.release()
        videos[cam] = frames
    return d, videos


def to_abs(state: np.ndarray, action: np.ndarray) -> np.ndarray:
    """ingest.py 와 동일 — 다음 프레임의 실측 pose + 이번 그리퍼 명령."""
    nxt = np.vstack([state[1:, :3], state[-1:, :3]])
    return np.concatenate([nxt, action[:, 3:4]], axis=1).astype(np.float32)


def episode_anchors(d: dict) -> tuple:
    """파지 좌표(obj_xy)·놓는 좌표(goal_xy)를 에피소드에서 뽑는다.

    obj_xy: CLOSE 첫 프레임의 EEF xy — 물체 위에서 닫으므로 물체 좌표와 같다.
            task3 v10 은 프레임별 target_x/y 가 있어 이건 예비값이다.
    goal_xy: 마지막 ALIGN/PLACE 프레임의 EEF xy (task2 단자 위치).
    """
    st, stages = d["state"], d["stage"]
    obj_xy = goal_xy = None
    for i, s in enumerate(stages):
        if s == "CLOSE":
            obj_xy = (float(st[i, 0]), float(st[i, 1]))
            break
    for i in range(len(stages) - 1, -1, -1):
        if stages[i] in ("ALIGN", "PLACE"):
            goal_xy = (float(st[i, 0]), float(st[i, 1]))
            break
    return obj_xy, goal_xy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--task", required=True, choices=tuple(TASK_TEXT))
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    src, dst = Path(args.src), Path(args.dst)
    if dst.exists():
        shutil.rmtree(dst)

    eps = [json.loads(l) for l in
           (src / "meta" / "episodes.jsonl").read_text().splitlines() if l.strip()]
    info = json.loads((src / "meta" / "info.json").read_text())
    fps = int(info["fps"])

    features = {
        "observation.state": {"dtype": "float32", "shape": (4,),
                              "names": ["x", "y", "z", "gripper"]},
        "action": {"dtype": "float32", "shape": (4,),
                   "names": ["x", "y", "z", "gripper"]},
    }
    for cam in CAMS:
        features[f"observation.images.{cam}"] = {
            "dtype": "video", "shape": (270, 480, 3),
            "names": ["height", "width", "channels"],
        }
    ds = LeRobotDataset.create(
        repo_id=f"local/{dst.name}", fps=fps, features=features,
        root=dst, robot_type="franka_robotiq_2f85",
    )

    style_count = dict.fromkeys(STYLES, 0)
    for ep in eps:
        idx = ep["episode_index"]
        # task 스타일 문장은 **에피소드 원문**이다 — task1/2 는 에피소드마다
        # 대상(망치/드릴, 붉은/검은 커넥터)이 달라 문장도 다르다. 전역 상수를
        # 쓰면 검은 커넥터 시연에 붉은 커넥터 문장이 붙는다.
        base = ep.get("tasks", [TASK_TEXT[args.task]])[0]
        d, videos = read_episode(src, idx)
        if d["stage"] is None:
            raise SystemExit(f"ep{idx}: stage 열이 없다 — *_src(v2.0) 원본을 넣어야 한다")
        act = to_abs(d["state"], d["action"])
        obj_fallback, goal_xy = episode_anchors(d)
        n = len(d["state"])
        assert all(len(videos[c]) == n for c in CAMS), \
            (idx, n, {c: len(videos[c]) for c in CAMS})
        rng = random.Random((args.seed << 20) ^ idx)   # 에피소드별 재현 가능
        for i in range(n):
            stage, target = d["stage"][i], d["target"][i]
            if d["txy"] is not None and np.isfinite(d["txy"][i]).all():
                obj_xy = (float(d["txy"][i, 0]), float(d["txy"][i, 1]))
            else:
                obj_xy = obj_fallback
            style = rng.choice(STYLES)
            text = None
            if style == "subtask":
                text = subtask_of(args.task, stage, target)
            elif style == "motion":
                text = motion_of(d["action"], i)
            elif style == "point":
                text = point_of(args.task, stage, target, obj_xy, goal_xy)
            if text is None:          # 그 단계에 그 스타일이 없으면 태스크 문장
                style = "task"
                text = base
            style_count[style] += 1
            ds.add_frame({
                "observation.state": d["state"][i],
                "action": act[i],
                **{f"observation.images.{c}": videos[c][i] for c in CAMS},
                "task": text,
            })
        ds.save_episode()
        print(f"  ep {idx}: {n}프레임", flush=True)

    ds.finalize()
    total = sum(style_count.values())
    print(f"완료: {dst} ({len(eps)} 에피소드, abs 4차원)", flush=True)
    print("명령 스타일 분포 — "
          + ", ".join(f"{k} {v} ({100.0 * v / total:.0f}%)"
                      for k, v in style_count.items()), flush=True)


if __name__ == "__main__":
    main()
