#!/usr/bin/env python3
"""수집기 v2.0 데이터셋을 lerobot 0.6.x 네이티브 형식으로 재적재한다.

우리 수집기는 v2.0 레이아웃(parquet + mp4 + jsonl)을 직접 쓰는데, lerobot 0.6 은
형식이 그보다 앞서 있고(통계·정규화 메타 포함) 버전 검사도 한다. 형식을 손으로
좇는 대신 **공식 API(LeRobotDataset.create / add_frame / save_episode)** 로 프레임을
다시 흘려 넣는다 — 무엇이 되어야 하는지는 lerobot 자신이 제일 잘 안다.

action 두 가지 변형을 같은 원본에서 만든다:
    delta  원본 그대로 — [dx dy dz droll dpitch dyaw grip] (로봇 명령 공간, 7차원)
    abs    다음 프레임의 EEF pose 절대값 — [x y z qw qx qy qz grip] (8차원).
           시연 궤적 자체가 목표가 된다. 마지막 프레임은 제자리(자기 pose)다.

사용:
    python ingest.py --src <v2.0 데이터셋> --dst <출력> --action delta|abs
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq

from lerobot.datasets.lerobot_dataset import LeRobotDataset

# 수집기와 같은 3종 (env/src/franka_env/camera.py 의 Top View 추가분 포함)
CAMS = ("front", "top", "wrist")


def read_episode(src: Path, idx: int):
    t = pq.read_table(src / "data" / "chunk-000" / f"episode_{idx:06d}.parquet")
    state = np.array(t.column("observation.state").to_pylist(), dtype=np.float32)
    action = np.array(t.column("action").to_pylist(), dtype=np.float32)
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
    return state, action, videos


def to_abs(state: np.ndarray, action: np.ndarray) -> np.ndarray:
    """절대값 action — 다음 프레임의 실측 pose + 이번 프레임의 그리퍼 명령.

    명령 델타를 현재 pose 에 더하는 방식은 쓰지 않는다. 팔이 명령의 ~30% 만
    따라가므로(상대 IK 실현률) 그 합은 실제로 간 적 없는 허공의 점이 된다.
    시연이 보여 준 것은 **실제로 지나간 궤적**이고, 절대 좌표 정책의 목표로는
    그쪽이 맞다.
    """
    nxt = np.vstack([state[1:, :3], state[-1:, :3]])       # 마지막은 제자리
    return np.concatenate([nxt, action[:, 3:4]], axis=1).astype(np.float32)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--action", choices=("delta", "abs"), required=True)
    args = ap.parse_args()
    src, dst = Path(args.src), Path(args.dst)
    if dst.exists():
        shutil.rmtree(dst)

    eps = [json.loads(l) for l in (src / "meta" / "episodes.jsonl").read_text().splitlines() if l.strip()]
    tasks = {t["task_index"]: t["task"] for t in
             (json.loads(l) for l in (src / "meta" / "tasks.jsonl").read_text().splitlines() if l.strip())}
    info = json.loads((src / "meta" / "info.json").read_text())
    fps = int(info["fps"])

    adim = 4
    anames = (["dx", "dy", "dz", "gripper"] if args.action == "delta"
              else ["x", "y", "z", "gripper"])
    features = {
        "observation.state": {"dtype": "float32", "shape": (4,),
                              "names": ["x", "y", "z", "gripper"]},
        "action": {"dtype": "float32", "shape": (adim,), "names": anames},
    }
    for cam in CAMS:
        features[f"observation.images.{cam}"] = {
            "dtype": "video", "shape": (270, 480, 3), "names": ["height", "width", "channels"],
        }

    ds = LeRobotDataset.create(
        repo_id=f"local/{dst.name}", fps=fps, features=features,
        root=dst, robot_type="franka_robotiq_2f85",
    )

    for ep in eps:
        idx = ep["episode_index"]
        task = ep.get("tasks", [tasks.get(0, "")])[0]
        state, action, videos = read_episode(src, idx)
        act = action if args.action == "delta" else to_abs(state, action)
        n = len(state)
        assert all(len(videos[c]) == n for c in CAMS), (idx, n, {c: len(videos[c]) for c in CAMS})
        for i in range(n):
            ds.add_frame({
                "observation.state": state[i],
                "action": act[i],
                **{f"observation.images.{c}": videos[c][i] for c in CAMS},
                "task": task,
            })
        ds.save_episode()
        print(f"  ep {idx}: {n}프레임", flush=True)

    ds.finalize()
    print(f"완료: {dst} ({len(eps)} 에피소드, action={args.action} {adim}차원)", flush=True)


if __name__ == "__main__":
    main()
