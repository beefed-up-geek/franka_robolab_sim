#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""학습한 VLA(GR00T N1.7) 추론 서버 — 호스트에서 GPU 로 돈다.

시뮬레이션 컨테이너 안의 rclpy(Isaac 번들)와 학습 스택(lerobot·torch)은
파이썬 환경이 달라 한 프로세스에 담기 어렵다. 그래서 RoboLab 이 쓰는
서버-클라이언트 구조를 따른다 (inference/README.md 참고):

    [컨테이너] run_policy.py  --HTTP-->  [호스트] policy_server.py
       ROS 관측 수집                        GR00T 추론 (GPU)
       액션 발행        <--액션 7차원--

실행:
    ~/hfenv/bin/python inference/policy_server.py --model <pretrained_model 경로>
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import torch
from PIL import Image

from lerobot.policies.factory import get_policy_class, make_pre_post_processors

CAMS = ("front", "top", "wrist")
_STATE = None


class Runner:
    def __init__(self, model_path: str, device: str = "cuda") -> None:
        cls = get_policy_class("groot")
        self.policy = cls.from_pretrained(model_path)
        self.policy.to(device)
        self.policy.eval()
        self.device = device
        self.n = 0
        # 학습 때 저장된 전·후처리 파이프라인(정규화·패킹)을 함께 싣는다 —
        # 이걸 빼면 정규화가 어긋나 액션이 엉뚱해진다 (lerobot_eval.py 와 동일 경로).
        self.pre, self.post = make_pre_post_processors(
            policy_cfg=self.policy.config,
            pretrained_path=model_path,
            preprocessor_overrides={"device_processor": {"device": device}},
        )
        # 모델이 실제로 요구하는 카메라만 쓴다 — 클라이언트는 항상 세 장을
        # 보내므로, 2카메라로 학습한 예전 모델도 그대로 돌아간다.
        self.cams = tuple(k.split(".")[-1] for k in self.policy.config.input_features
                          if k.startswith("observation.images."))
        if not self.cams:
            self.cams = CAMS
        print(f"[server] 정책·전후처리 로드 완료: {model_path} | 카메라 {self.cams}",
              flush=True)

    def reset(self) -> None:
        self.policy.reset()
        self.n = 0

    @torch.no_grad()
    def act(self, state, images: dict, task: str):
        batch = {
            "observation.state": torch.tensor(
                state, dtype=torch.float32).unsqueeze(0),
            "task": [task],
        }
        for cam in self.cams:
            arr = np.asarray(images[cam], dtype=np.float32) / 255.0     # HWC RGB
            t = torch.from_numpy(arr).permute(2, 0, 1).unsqueeze(0)     # 1,C,H,W
            batch[f"observation.images.{cam}"] = t
        obs = self.pre(batch)
        act = self.policy.select_action(obs)
        act = self.post(act)
        self.n += 1
        return act.squeeze(0).float().cpu().numpy().tolist()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a) -> None:      # 접속 로그는 끈다 — 6Hz 로 시끄럽다
        pass

    def _send(self, obj, code=200) -> None:
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        n = int(self.headers.get("Content-Length", 0))
        req = json.loads(self.rfile.read(n) or b"{}")
        if self.path == "/reset":
            _STATE.reset()
            return self._send({"ok": True})
        if self.path != "/act":
            return self._send({"error": "unknown path"}, 404)
        t0 = time.time()
        images = {c: Image.open(io.BytesIO(base64.b64decode(req["images"][c]))).convert("RGB")
                  for c in CAMS}
        action = _STATE.act(req["state"], images, req.get("task", ""))
        self._send({"action": action, "ms": round((time.time() - t0) * 1000, 1)})


def main() -> None:
    ap = argparse.ArgumentParser(description="GR00T 추론 서버")
    ap.add_argument("--model", required=True, help="pretrained_model 디렉터리")
    ap.add_argument("--port", type=int, default=8010)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--action-steps", type=int, default=0,
                    help="한 번 추론으로 실행할 액션 수. 0=학습값 그대로(16). "
                         "삽입처럼 정밀한 작업은 4 이하로 줄여 자주 다시 본다.")
    args = ap.parse_args()

    global _STATE
    _STATE = Runner(args.model, args.device)
    if args.action_steps:
        _STATE.policy.config.n_action_steps = args.action_steps
        print(f"[server] 액션 청크 {args.action_steps} 스텝으로 축소", flush=True)
    srv = HTTPServer(("127.0.0.1", args.port), Handler)
    print(f"[server] 준비 완료 — http://127.0.0.1:{args.port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
