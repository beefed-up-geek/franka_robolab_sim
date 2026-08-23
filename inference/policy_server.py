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
import math
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
        self._aff = None            # 정규화 해제 아핀 계수 (지연 역산)
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
        self.act_dim = int(self.policy.config.action_feature.shape[0]) \
            if getattr(self.policy.config, "action_feature", None) is not None else 4
        print(f"[server] 정책·전후처리 로드 완료: {model_path} | 카메라 {self.cams}",
              flush=True)

    def reset(self) -> None:
        self.policy.reset()
        self.n = 0

    def _batch(self, state, images: dict, task: str) -> dict:
        batch = {
            "observation.state": torch.tensor(
                state, dtype=torch.float32).unsqueeze(0),
            "task": [task],
        }
        for cam in self.cams:
            arr = np.asarray(images[cam], dtype=np.float32) / 255.0     # HWC RGB
            t = torch.from_numpy(arr).permute(2, 0, 1).unsqueeze(0)     # 1,C,H,W
            batch[f"observation.images.{cam}"] = t
        return batch

    @torch.no_grad()
    def act(self, state, images: dict, task: str):
        obs = self.pre(self._batch(state, images, task))
        act = self.policy.select_action(obs)
        act = self.post(act)
        self.n += 1
        return act.squeeze(0).float().cpu().numpy().tolist()

    @torch.no_grad()
    def act_chunk(self, state, images: dict, task: str, k: int):
        """VLS 용 — flow 헤드에서 후보 청크 K 개를 뽑는다 (입자 다양성).

        같은 관측에 대한 K 번의 독립 샘플이다. flow matching 은 노이즈에서
        출발하므로 매 호출이 다른 궤적을 낸다. select_action 의 내부 액션
        큐는 건드리지 않아 /act 경로와 간섭하지 않는다. 후처리(정규화 해제)는
        (1,T,A) 텐서에 그대로 적용된다 — unnormalize 가 마지막 축만 본다.
        """
        chunks = []
        for _ in range(max(1, min(int(k), 16))):
            obs = self.pre(self._batch(state, images, task))
            ch = self.policy.predict_action_chunk(obs)      # (1, T, A)
            ch = self.post(ch)
            chunks.append(ch.squeeze(0).float().cpu().numpy().tolist())
        self.n += 1
        return chunks

    # ── VLS_authentic — 논문 Algorithm 1 의 유도 디노이징 ────────────────
    def _affine(self):
        """정규화 해제의 아핀 계수 (world = norm * scale + offset).

        모드를 추측하지 않고 두 점(0, 1)을 후처리에 통과시켜 역산한다 —
        min-max 든 mean-std 든 아핀이기만 하면 정확하다 (실측 오차 3e-8).
        """
        if self._aff is None:
            T = self.policy.config.chunk_size
            probe = lambda v: self.post(torch.full((1, T, self.act_dim), float(v)))
            o0, o1 = probe(0.0), probe(1.0)
            self._aff = ((o1 - o0)[0, 0].clone(), o0[0, 0].clone())
        return self._aff

    def act_chunk_guided(self, state, images: dict, task: str, *, n_particles: int,
                         reward_src: str, kp: dict, lam: float, mcmc_steps: int,
                         guidance_lr: float, rbf_weight: float, fk_temp: float,
                         tcp_dz: float, max_dev: float):
        """논문 Alg.1 을 그대로 — 디노이징 루프 **안에서** 유도한다.

        VLS(vanilla)가 완성된 표본을 고른 뒤 다듬는 것과 달리, 여기서는
        매 디노이징 스텝마다 ① RBF 반발로 입자를 흩고 ② 보상 기울기를
        주입하고(MCMC 내부 갱신) ③ Feynman-Kac 가중으로 리샘플링한다.

        보상은 월드(TCP) 좌표에서 정의되므로 정규화 공간의 액션을 아핀으로
        펴서 평가하고, 기울기는 연쇄법칙으로 되돌린다. bf16 루프 안에서
        미분하면 정밀도가 무너져 유도 계산만 fp32 로 올린다.
        """
        head = self.policy._groot_model.action_head
        scale, offset = self._affine()
        dev = next(self.policy.parameters()).device
        sc = scale.to(dev).float()
        off = offset.to(dev).float()
        tcp = torch.zeros_like(sc)
        tcp[2] = tcp_dz                      # 플랜지 → TCP (z 만)

        ns = {"torch": torch, "math": math, "__builtins__": {
            "len": len, "min": min, "max": max, "abs": abs, "float": float,
            "sum": sum, "range": range, "True": True, "False": False,
           # torch.tensor 는 내부에서 torch.storage 를 import 한다 — __import__
           # 이 없으면 "storage_module && PyModule_Check" INTERNAL ASSERT 로
           # 죽는다 (Isaac 쪽 torch 에서 실측). 보상 코드가 거의 항상
           # torch.tensor(kp[...]) 를 쓰므로 반드시 열어줘야 한다.
           "__import__": __import__}}
        exec("def _r(traj, kp):\n" + "\n".join(          # noqa: S102 — 연구용
            "    " + ln for ln in reward_src.strip().splitlines()), ns)
        reward_fn = ns["_r"]

        nd = int(sc.shape[0])                # 실제 액션 차원 (패딩 이전)

        diag = {"resamples": 0, "r_first": None, "r_last": None}

        def batch_reward(a_norm):
            """(B,T,A_pad) 정규화 액션 → (B,) 보상. 그래프를 남긴다.

            GR00T 는 다중 임베디먼트를 위해 액션을 132차원으로 패딩해서
            디노이징한다. 후처리가 앞 env_action_dim(=4) 만 잘라 쓰므로
            (processor_groot: action[..., :env_action_dim]) 유도도 같은
            슬라이스에만 걸어야 한다 — 패딩 축에 기울기를 주면 아무 데도
            안 쓰이는 좌표를 미는 셈이다.
            """
            # 보상은 **CPU** 에서 평가한다 — VLM 이 낸 코드가
            # torch.tensor(kp["..."]) 로 CPU 텐서를 만들기 때문이다. 궤적은
            # (T,4) 로 작아 전송·연산 비용이 무시할 수준이고, 기울기는
            # autograd 가 GPU 쪽 그래프로 그대로 되돌린다.
            w = a_norm[..., :nd].float().cpu() * sc.cpu() + off.cpu()
            w = torch.cat([w[..., :3] + tcp[:3].cpu(), w[..., 3:]], dim=-1)
            # VLM 이 쓴 보상이 던지는 예외는 여기서 삼킨다. 예외가 핸들러까지
            # 올라가면 연결이 끊겨(RemoteDisconnected) 클라이언트가 vanilla 로
            # 되돌아가고, 그 뒤로 그 에피소드 내내 유도가 사라진다 — 실측으로
            # VLSa/task3 가 통째로 무유도 실행됐다(1334 회). 실패는 진단에
            # 남기고 평탄한 보상(0)으로 계속한다.
            out = []
            for i in range(w.shape[0]):
                try:
                    out.append(reward_fn(w[i], kp))
                except Exception as e:           # noqa: BLE001
                    diag["reward_err"] = f"{type(e).__name__}: {e}"
                    out.append(w[i].sum() * 0.0)
            return torch.stack(out)

        orig = head.get_action_with_features
        B = max(2, min(int(n_particles), 16))

        def guided(backbone_features, state_features, embodiment_id,
                   backbone_output, action_input, options=None):
            vl = backbone_features
            if vl.shape[0] == 1:
                vl = vl.expand(B, *vl.shape[1:]).contiguous()
            sf = state_features
            if sf.shape[0] == 1:
                sf = sf.expand(B, *sf.shape[1:]).contiguous()
            emb = embodiment_id
            if emb.shape[0] == 1:
                emb = emb.expand(B).contiguous()
            im = getattr(backbone_output, "image_mask", None)
            am = getattr(backbone_output, "backbone_attention_mask", None)
            if im is not None and im.shape[0] == 1:
                im = im.expand(B, *im.shape[1:]).contiguous()
            if am is not None and am.shape[0] == 1:
                am = am.expand(B, *am.shape[1:]).contiguous()

            steps = head.num_inference_timesteps
            actions = torch.randn(B, head.config.action_horizon, head.action_dim,
                                  dtype=vl.dtype, device=vl.device)
            dt = 1.0 / steps
            for t_step in range(steps):
                t_cont = t_step / float(steps)
                t_disc = int(t_cont * head.num_timestep_buckets)
                ts = torch.full((B,), t_disc, device=vl.device)
                af = head.action_encoder(actions, ts, emb)
                if head.config.add_pos_embed:
                    pos = torch.arange(af.shape[1], dtype=torch.long, device=vl.device)
                    af = af + head.position_embedding(pos).unsqueeze(0)
                sa = torch.cat((sf, af), dim=1)
                if head.config.use_alternate_vl_dit:
                    mo = head.model(hidden_states=sa, encoder_hidden_states=vl,
                                    timestep=ts, image_mask=im,
                                    backbone_attention_mask=am)
                else:
                    mo = head.model(hidden_states=sa, encoder_hidden_states=vl,
                                    timestep=ts)
                pred = head.action_decoder(mo, emb)
                actions = actions + dt * pred[:, -head.config.action_horizon:]

                # ① 다양성 — RBF 반발 (eq.8). 초기 스텝에서만 세게 준다.
                if rbf_weight > 0 and t_step < max(1, steps // 2):
                    with torch.no_grad():
                        a = actions.float()
                        d = a.unsqueeze(1) - a.unsqueeze(0)         # (B,B,T,A)
                        dist = d.flatten(2).norm(dim=2) + 1e-6      # (B,B)
                        wgt = 1.0 / (dist * (dist + 1e-6) ** 2)
                        wgt.fill_diagonal_(0.0)
                        rep = (d.flatten(2) * wgt.unsqueeze(2)).sum(1)
                        rep = rep / (rep.norm(dim=1, keepdim=True) + 1e-8)
                        actions = actions + (rbf_weight * rep.view_as(a)).to(actions.dtype)

                # ② 보상 기울기 주입 — MCMC 내부 갱신 (Alg.1 14-16)
                #
                # **변위 상한이 핵심이다.** 상한 없이 λ·lr·(MCMC 횟수)를 곱하면
                # 누적 이동이 정규화 액션 범위(±1)를 통째로 넘어서 궤적이
                # 정책 매니폴드 밖으로 날아간다 (실측: 파지 0회, λ 가 2.0 에
                # 고정된 채 폭주). 스텝당 상한을 max_dev/steps 로 두어 한
                # 에피소드의 총 유도량이 max_dev 를 넘지 않게 한다 —
                # VLS(vanilla)의 REFINE_CLAMP(±4cm)와 같은 취지다.
                cap = max_dev / max(1, steps)
                if mcmc_steps > 0:
                    pre = actions.float().clone()
                    for _ in range(max(0, int(mcmc_steps))):
                        with torch.enable_grad():
                            a = actions.float().detach().requires_grad_(True)
                            r = batch_reward(a)
                            (g,) = torch.autograd.grad(r.sum(), a)
                        g = g / (g.flatten(1).norm(dim=1).view(-1, 1, 1) + 1e-8)
                        actions = (actions.float() + lam * guidance_lr * g).to(actions.dtype)
                    dev = (actions.float() - pre).clamp(-cap, cap)
                    actions = (pre + dev).to(actions.dtype)

                # ③ Feynman-Kac 리샘플링 (eq.9) — 마지막 스텝 직전까지
                if t_step < steps - 1:
                    with torch.no_grad():
                        r = batch_reward(actions.float())
                        w = torch.softmax(r / max(fk_temp, 1e-6), dim=0)
                        idx = torch.multinomial(w, B, replacement=True)
                        if not bool((idx == torch.arange(B, device=idx.device)).all()):
                            diag["resamples"] += 1
                        actions = actions[idx].contiguous()

            with torch.no_grad():
                r = batch_reward(actions.float())
                diag["r_first"] = float(r.max())
                best = int(r.argmax())
                actions = actions[best:best + 1].contiguous()
            from transformers.feature_extraction_utils import BatchFeature
            return BatchFeature(data={"action_pred": actions})

        head.get_action_with_features = guided
        try:
            with torch.no_grad():
                obs = self.pre(self._batch(state, images, task))
                ch = self.policy.predict_action_chunk(obs)
            ch = self.post(ch)
        finally:
            head.get_action_with_features = orig
        self.n += 1
        return (ch.squeeze(0).float().cpu().numpy().tolist(), diag)


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
        if self.path not in ("/act", "/act_chunk", "/act_chunk_guided"):
            return self._send({"error": "unknown path"}, 404)
        t0 = time.time()
        images = {c: Image.open(io.BytesIO(base64.b64decode(req["images"][c]))).convert("RGB")
                  for c in CAMS}
        if self.path == "/act_chunk_guided":
            chunk, diag = _STATE.act_chunk_guided(
                req["state"], images, req.get("task", ""),
                n_particles=req.get("n_particles", 6),
                reward_src=req["reward_src"], kp=req["kp"],
                lam=req.get("lam", 1.0), mcmc_steps=req.get("mcmc_steps", 2),
                guidance_lr=req.get("guidance_lr", 0.05),
                rbf_weight=req.get("rbf_weight", 0.02),
                fk_temp=req.get("fk_temp", 1.0),
                tcp_dz=req.get("tcp_dz", -0.15),
                max_dev=req.get("max_dev", 0.12))
            return self._send({"chunk": chunk, "diag": diag,
                               "ms": round((time.time() - t0) * 1000, 1)})
        if self.path == "/act_chunk":
            chunks = _STATE.act_chunk(req["state"], images, req.get("task", ""),
                                      req.get("k", 4))
            return self._send({"chunks": chunks,
                               "ms": round((time.time() - t0) * 1000, 1)})
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
