# SPDX-License-Identifier: Apache-2.0
"""수집한 에피소드를 LeRobot 형식으로 저장한다.

`lerobot` 패키지를 쓰지 않고 직접 쓴다. 설치하면 torch/datasets 등 무거운 의존성이
따라오고 버전이 얽히는데, 저장 형식 자체는 parquet + mp4 + 메타 JSON 이라 그럴
이유가 없다.

    <root>/
      meta/info.json          형식·주기·피처 정의
      meta/tasks.jsonl        태스크 문장
      meta/episodes.jsonl     에피소드별 길이·태스크
      data/chunk-000/episode_XXXXXX.parquet     상태·액션 시계열
      videos/chunk-000/observation.images.<cam>/episode_XXXXXX.mp4

프레임 인덱스와 타임스탬프는 **에피소드 안에서 0부터** 시작하고, `index` 만
데이터셋 전체에서 이어진다. LeRobot 이 그렇게 읽는다.
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

CHUNK = "chunk-000"


class LeRobotWriter:
    def __init__(self, root: str | Path, fps: float, cameras: list[str],
                 state_dim: int, action_dim: int, task: str) -> None:
        self.root = Path(root)
        self.fps = float(fps)
        self.cameras = list(cameras)
        self.task = task
        self.state_dim, self.action_dim = state_dim, action_dim

        (self.root / "meta").mkdir(parents=True, exist_ok=True)
        (self.root / "data" / CHUNK).mkdir(parents=True, exist_ok=True)
        for cam in self.cameras:
            (self.root / "videos" / CHUNK / f"observation.images.{cam}").mkdir(
                parents=True, exist_ok=True)

        # 이어쓰기 — 이미 있는 에피소드 뒤에 붙인다.
        self.episodes = self._load_jsonl("meta/episodes.jsonl")
        self.ep_index = len(self.episodes)
        self.total_frames = sum(e["length"] for e in self.episodes)

        self._frames: list[dict] = []
        self._video: dict[str, list[np.ndarray]] = {c: [] for c in self.cameras}

    # ── 수집 ──────────────────────────────────────────────────────────
    def add(self, state, action, images: dict[str, bytes], extra: dict | None = None) -> None:
        """한 스텝. images 는 JPEG 바이트 그대로 받는다."""
        row = {
            "observation.state": [float(v) for v in state],
            "action": [float(v) for v in action],
        }
        row.update(extra or {})
        self._frames.append(row)
        for cam in self.cameras:
            jpeg = images.get(cam)
            if jpeg:
                arr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            else:
                arr = None
            # 프레임이 빠지면 직전 것을 복제한다. 영상 길이와 표 길이가 어긋나면
            # LeRobot 이 읽을 때 인덱스가 밀린다.
            if arr is None:
                arr = self._video[cam][-1] if self._video[cam] else np.zeros((270, 480, 3), np.uint8)
            self._video[cam].append(arr)

    def discard(self) -> None:
        self._frames.clear()
        for c in self.cameras:
            self._video[c].clear()

    # ── 저장 ──────────────────────────────────────────────────────────
    def save_episode(self) -> int | None:
        n = len(self._frames)
        if n == 0:
            return None
        idx = self.ep_index
        cols = {
            "observation.state": [f["observation.state"] for f in self._frames],
            "action": [f["action"] for f in self._frames],
            "timestamp": [i / self.fps for i in range(n)],
            "frame_index": list(range(n)),
            "episode_index": [idx] * n,
            "index": list(range(self.total_frames, self.total_frames + n)),
            "task_index": [0] * n,
        }
        for key in self._frames[0]:
            if key not in cols:
                cols[key] = [f.get(key) for f in self._frames]
        pq.write_table(pa.table(cols),
                       self.root / "data" / CHUNK / f"episode_{idx:06d}.parquet")

        for cam in self.cameras:
            self._write_video(cam, idx)

        self.episodes.append({"episode_index": idx, "tasks": [self.task], "length": n})
        self._write_jsonl("meta/episodes.jsonl", self.episodes)
        self._write_jsonl("meta/tasks.jsonl", [{"task_index": 0, "task": self.task}])
        self.total_frames += n
        self.ep_index += 1
        self._write_info()
        self.discard()
        return idx

    def prune_episodes(self, bad: list[int]) -> None:
        """에피소드들을 지우고 남은 것을 앞으로 당겨 번호를 다시 매긴다.

        수집이 끝난 뒤 "다른 것보다 너무 오래 걸린" 에피소드를 걷어내는 용도다.
        LeRobot 은 에피소드 번호가 0부터 빈틈없이 이어진다고 가정하므로 파일만
        지우면 안 되고, 남은 에피소드의 parquet 안에 박힌 episode_index 와
        전체 누적 index 열까지 다시 써야 한다.
        """
        bad_set = set(bad)
        keep = [e for e in self.episodes if e["episode_index"] not in bad_set]

        for idx in bad_set:
            (self.root / "data" / CHUNK / f"episode_{idx:06d}.parquet").unlink(missing_ok=True)
            for cam in self.cameras:
                (self.root / "videos" / CHUNK / f"observation.images.{cam}"
                 / f"episode_{idx:06d}.mp4").unlink(missing_ok=True)

        total = 0
        new_eps = []
        for new_idx, ep in enumerate(keep):
            old_idx = ep["episode_index"]
            n = ep["length"]
            if new_idx != old_idx:
                # 앞으로 당겨진 에피소드 — parquet 의 번호 열을 고쳐 다시 쓴다.
                # new_idx < old_idx 이고 오름차순으로 처리하므로 목적지 파일은
                # 언제나 이미 지워졌거나 이미 옮겨 간 자리라 덮어쓸 걱정이 없다.
                src = self.root / "data" / CHUNK / f"episode_{old_idx:06d}.parquet"
                dst = self.root / "data" / CHUNK / f"episode_{new_idx:06d}.parquet"
                t = pq.read_table(src)
                t = t.set_column(t.schema.get_field_index("episode_index"),
                                 "episode_index", pa.array([new_idx] * n, pa.int64()))
                t = t.set_column(t.schema.get_field_index("index"),
                                 "index", pa.array(range(total, total + n), pa.int64()))
                pq.write_table(t, dst)
                src.unlink()
                for cam in self.cameras:
                    d = self.root / "videos" / CHUNK / f"observation.images.{cam}"
                    (d / f"episode_{old_idx:06d}.mp4").replace(
                        d / f"episode_{new_idx:06d}.mp4")
            # new_idx == old_idx 면 앞에서 지워진 것이 없다는 뜻이라 누적 index 도
            # 그대로다 — 손댈 것이 없다.
            total += n
            new_eps.append({"episode_index": new_idx, "tasks": ep["tasks"], "length": n})

        self.episodes = new_eps
        self.ep_index = len(new_eps)
        self.total_frames = total
        self._write_jsonl("meta/episodes.jsonl", self.episodes)
        self._write_info()

    def _write_video(self, cam: str, idx: int) -> None:
        frames = self._video[cam]
        if not frames:
            return
        h, w = frames[0].shape[:2]
        path = (self.root / "videos" / CHUNK / f"observation.images.{cam}"
                / f"episode_{idx:06d}.mp4")
        # avc1 은 컨테이너에 코덱이 없을 수 있어 mp4v 로 떨어뜨린다.
        for fourcc in ("avc1", "mp4v"):
            vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*fourcc), self.fps, (w, h))
            if vw.isOpened():
                for f in frames:
                    vw.write(f if f.shape[:2] == (h, w) else cv2.resize(f, (w, h)))
                vw.release()
                return
        raise RuntimeError(f"영상 인코더를 열지 못했습니다: {path}")

    def _write_info(self) -> None:
        feats = {
            "observation.state": {"dtype": "float32", "shape": [self.state_dim],
                                  "names": STATE_NAMES[:self.state_dim]},
            "action": {"dtype": "float32", "shape": [self.action_dim],
                       "names": ACTION_NAMES[:self.action_dim]},
            "timestamp": {"dtype": "float32", "shape": [1], "names": None},
            "frame_index": {"dtype": "int64", "shape": [1], "names": None},
            "episode_index": {"dtype": "int64", "shape": [1], "names": None},
            "index": {"dtype": "int64", "shape": [1], "names": None},
            "task_index": {"dtype": "int64", "shape": [1], "names": None},
        }
        for cam in self.cameras:
            feats[f"observation.images.{cam}"] = {
                "dtype": "video", "shape": [270, 480, 3],
                "names": ["height", "width", "channel"],
                "info": {"video.fps": self.fps, "video.codec": "mp4v",
                         "video.is_depth_map": False, "has_audio": False},
            }
        info = {
            "codebase_version": "v2.0",
            "robot_type": "franka_robotiq_2f85",
            "total_episodes": len(self.episodes),
            "total_frames": self.total_frames,
            "total_tasks": 1,
            "total_videos": len(self.episodes) * len(self.cameras),
            "total_chunks": 1,
            "chunks_size": 1000,
            "fps": self.fps,
            "splits": {"train": f"0:{len(self.episodes)}"},
            # 실제 디렉터리는 data/chunk-000 이다. "chunk-" 접두사를 빼먹으면
            # LeRobot 이 data/000 을 찾다가 파일을 못 열어 로드가 통째로 실패한다.
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features": feats,
        }
        (self.root / "meta" / "info.json").write_text(
            json.dumps(info, indent=2, ensure_ascii=False))

    # ── 파일 ──────────────────────────────────────────────────────────
    def _load_jsonl(self, rel: str) -> list[dict]:
        p = self.root / rel
        if not p.exists():
            return []
        return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]

    def _write_jsonl(self, rel: str, rows: list[dict]) -> None:
        (self.root / rel).write_text(
            "\n".join(json.dumps(r, ensure_ascii=False) for r in rows) + "\n")


STATE_NAMES = ["eef_x", "eef_y", "eef_z", "eef_qw", "eef_qx", "eef_qy", "eef_qz", "gripper"]
ACTION_NAMES = ["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"]
