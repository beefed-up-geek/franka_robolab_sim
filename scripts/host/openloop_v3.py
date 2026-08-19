#!/usr/bin/env python3
"""개루프 검증 — 데이터셋 프레임을 그대로 정책에 넣고 기록된 액션과 비교한다.

    ~/hfenv/bin/python ~/openloop_v3.py <pretrained_model> <lerobot 데이터셋 폴더> [에피소드수]
"""
import sys, numpy as np, torch, pandas as pd, glob, io
from PIL import Image
from lerobot.policies.factory import get_policy_class, make_pre_post_processors

model_path, data_dir = sys.argv[1], sys.argv[2]
n_eps = int(sys.argv[3]) if len(sys.argv) > 3 else 2

cls = get_policy_class("groot")
policy = cls.from_pretrained(model_path); policy.to("cuda"); policy.eval()
pre, post = make_pre_post_processors(policy_cfg=policy.config, pretrained_path=model_path,
    preprocessor_overrides={"device_processor": {"device": "cuda"}})
cams = tuple(k.split(".")[-1] for k in policy.config.input_features if k.startswith("observation.images."))
print("카메라", cams)

df = pd.read_parquet(glob.glob(f"{data_dir}/data/chunk-000/*.parquet")[0])
import json
with open(f"{data_dir}/meta/info.json") as f: info = json.load(f)
fps = info["features"]["observation.images.front"]["info"]["video.fps"] if "info" in info["features"]["observation.images.front"] else 6

# 비디오에서 프레임 꺼내기
import av
def frames_of(ep, cam, idxs):
    vpath = glob.glob(f"{data_dir}/videos/observation.images.{cam}/chunk-000/*.mp4")
    vpath = sorted(vpath)[0]
    cont = av.open(vpath)
    stream = cont.streams.video[0]
    out = {}
    want = set(idxs)
    for i, frame in enumerate(cont.decode(stream)):
        if i in want:
            out[i] = frame.to_ndarray(format="rgb24")
            if len(out) == len(want): break
    cont.close()
    return out

t = pd.read_parquet(f"{data_dir}/meta/tasks.parquet")
task_texts = {int(r["task_index"]): idx for idx, r in t.iterrows()}

for ep in range(n_eps):
    d = df[df["episode_index"] == ep].reset_index(drop=True)
    glob_idx = d.index  # 파일 내 프레임 = 전역 인덱스로 비디오 접근
    base = df[df["episode_index"] < ep].shape[0]
    sample = list(range(0, len(d), max(1, len(d)//12)))
    vid_idx = [base + i for i in sample]
    imgs = {c: frames_of(ep, c, vid_idx) for c in cams}
    errs, coss, gok = [], [], 0
    for k, i in enumerate(sample):
        st = np.array(d["observation.state"][i], dtype=np.float32)
        gt = np.array(d["action"][i], dtype=np.float32)
        batch = {"observation.state": torch.tensor(st).unsqueeze(0), "task": [task_texts[int(d["task_index"][i])]]}
        for c in cams:
            arr = imgs[c][base + i].astype(np.float32) / 255.0
            batch[f"observation.images.{c}"] = torch.from_numpy(arr).permute(2,0,1).unsqueeze(0)
        policy.reset()
        with torch.no_grad():
            act = post(policy.select_action(pre(batch)))
        pr = act.squeeze(0).float().cpu().numpy()
        e = pr[:3] - gt[:3]
        errs.append(np.linalg.norm(e))
        na, nb = np.linalg.norm(pr[:3]), np.linalg.norm(gt[:3])
        if na > 1e-8 and nb > 1e-8: coss.append(float(np.dot(pr[:3], gt[:3])/(na*nb)))
        gok += int((pr[3] > 0.5) == (gt[3] > 0.5))
    print(f"ep{ep}: 프레임 {len(sample)}개 | xyz 오차 평균 {np.mean(errs):.4f} 최대 {np.max(errs):.4f} | 방향 코사인 {np.mean(coss):.3f} | 그리퍼 일치 {gok}/{len(sample)}")
