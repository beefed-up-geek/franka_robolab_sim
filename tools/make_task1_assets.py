#!/usr/bin/env python3
"""task1 주의 테이프·구역 텍스처를 그린다 → env/asset/textures/

    python3 tools/make_task1_assets.py

테이프는 노랑/검정 45° 사선(산업 주의 표기 관례). 빨강/검정 변형도 함께 만들어
씬에서 파일 이름만 바꾸면 전환된다.
"""
from pathlib import Path

from PIL import Image, ImageDraw

OUT = Path(__file__).resolve().parents[1] / "env/asset/textures"


def stripes(path: Path, a, b, w=512, h=64, period=64) -> None:
    img = Image.new("RGB", (w, h), a)
    d = ImageDraw.Draw(img)
    # 45° 사선 — 폭 절반씩 두 색. 좌우로 이어 붙어도 무늬가 이어진다.
    for x0 in range(-h - period, w + period, period):
        d.polygon([(x0, h), (x0 + h, 0), (x0 + h + period // 2, 0),
                   (x0 + period // 2, h)], fill=b)
    img.save(path, optimize=True)
    print("생성:", path.name, img.size)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    stripes(OUT / "caution_yellow.png", (232, 195, 30), (18, 18, 22))
    stripes(OUT / "caution_red.png", (200, 55, 45), (18, 18, 22))
    # 작업자 구역 면 — 옅은 사선 (상판보다 살짝 밝은 노랑기, 카메라에서 구분용)
    img = Image.new("RGB", (512, 512), (233, 229, 210))
    d = ImageDraw.Draw(img)
    for x0 in range(-512, 1024, 64):
        d.polygon([(x0, 512), (x0 + 512, 0), (x0 + 512 + 10, 0), (x0 + 10, 512)],
                  fill=(214, 205, 160))
    img.save(OUT / "worker_zone.png", optimize=True)
    print("생성: worker_zone.png")


if __name__ == "__main__":
    main()
