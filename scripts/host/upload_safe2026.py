#!/usr/bin/env python3
"""safe-2026 — 최종 abs VLA 6종을 폴더별로 한 리포지토리에 업로드한다."""
import os, sys, time
from huggingface_hub import HfApi

REPO = "nullPointerExcept1on/safe-2026"
HOME = os.path.expanduser("~")
# (리포 안 폴더, 로컬 모델) — vanilla 는 태스크별 최종 abs 학습본이고,
# lang 은 VLA_lang(steering command, 역시 abs 액션) 학습본이다.
JOBS = [
    ("task1_vanilla", "task1_abs"),
    ("task1_lang",    "task1_lang"),
    ("task2_vanilla", "task2_abs"),
    ("task2_lang",    "task2_lang"),
    ("task3_vanilla", "task3_abs_v10"),
    ("task3_lang",    "task3_lang_v10"),
]

README = """---
license: apache-2.0
tags: [robotics, vla, gr00t, lerobot, franka]
---
# safe-2026 — Franka 시뮬레이션 VLA (abs) 6종

GR00T N1.7-3B 를 lerobot 0.6.2 로 미세조정한 절대좌표(abs, [x y z gripper]) 정책들.
과제·환경·수집기는 https://github.com/beefed-up-geek/franka_robolab_sim 참고.

| 폴더 | 과제 | 학습 데이터 |
|---|---|---|
| task1_vanilla | 공구 전달 | task1 abs 200에피 |
| task1_lang | 공구 전달 | 위 + steering command 합성 (Steerable Policies, arXiv:2602.13193) |
| task2_vanilla | 충전 커넥터 연결 | task2 abs 200에피 |
| task2_lang | 충전 커넥터 연결 | 위 + steering command 합성 |
| task3_vanilla | 캔 분류 (저속 컨베이어 v10) | task3 abs_v10 200에피 |
| task3_lang | 캔 분류 (저속 컨베이어 v10) | 위 + steering command 합성 |

lang 모델은 task 문장 외에 subtask("grasp the can"), 원자 동작("move left"),
**월드 좌표 지시**("reach for the can at [0.52, -0.20]") 를 따른다.
각 폴더가 lerobot `pretrained_model` 형식 그대로라 폴더 경로를 모델 경로로 쓰면 된다.
"""

def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)

api = HfApi(token=os.environ["HF_TOKEN"])
api.create_repo(REPO, repo_type="model", private=True, exist_ok=True)
log("리포 준비:", REPO)
api.upload_file(path_or_fileobj=README.encode(), path_in_repo="README.md",
                repo_id=REPO, repo_type="model")
log("README 업로드")
for sub, key in JOBS:
    src = f"{HOME}/_model/{key}/pretrained_model"
    if not os.path.isfile(f"{src}/model.safetensors"):
        log("✗ 없음:", key); sys.exit(1)
    log(f"업로드 시작: {key} → {sub}/")
    api.upload_folder(folder_path=src, path_in_repo=sub,
                      repo_id=REPO, repo_type="model",
                      commit_message=f"add {sub} (from {key})")
    log(f"완료: {sub}/")
log("전체 완료")
open(os.path.expanduser("~/upload_safe2026.done"), "w").write("ok")
