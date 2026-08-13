#!/usr/bin/env python3
"""캔 라벨 이미지를 그린다 — tools/can_designs.py 의 도안 표를 보고.

    python3 tools/make_can_labels.py        # env/asset/objects/cans/textures/*.png

라벨은 몸통을 **한 바퀴 감는다.** 가로가 둘레, 세로가 라벨 띠 높이다. 그래서
구도를 잡을 때 두 가지를 지켜야 한다.

1. 이음매(u=0/1)를 가로지르는 것은 뒷면 정보(중량·바코드)뿐이다. 앞면 도안이
   이음매에 걸리면 캔을 어느 쪽에서 보든 반드시 잘린 곳이 보인다.
2. 이음매를 넘는 요소는 좌우 양쪽에 **모두** 그려야 한다. `_wrapped()` 가 같은
   그리기를 -W / 0 / +W 로 세 번 호출해 그 일을 한다.

이 이미지는 알베도(diffuse)다. 명암을 구워 넣으면 안 된다 — 렌더러가 조명을
얹는데 그림자가 이미 칠해져 있으면 빛 방향과 어긋나 평평해 보인다. 그래서 무늬는
색으로만 넣고 그라데이션 음영은 쓰지 않는다.

PIL 이 필요하다. 형상 생성기(make_cans.py)는 의존성이 없어야 해서 따로 뒀다.
"""
from __future__ import annotations

import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from PIL import Image, ImageDraw, ImageFont          # noqa: E402

from can_designs import DESIGNS, LABEL_H, LABEL_W    # noqa: E402

FONTS = Path("/usr/share/fonts/truetype/dejavu")
SERIF = FONTS / "DejaVuSerif-Bold.ttf"
SANS = FONTS / "DejaVuSans-Bold.ttf"
SANS_C = FONTS / "DejaVuSansCondensed-Bold.ttf"

# 라벨 위아래 테두리 — 실물 통조림은 라벨 끝에 괘선이 들어가 몸통 강판과 갈린다.
EDGE = 13          # 바깥 굵은 괘선 [px]
INNER = 11         # 그 안쪽 얇은 띠 [px]

# 앞면 도안이 차지하는 가로 구간. 이음매(0/W)에서 충분히 떨어뜨린다.
FRONT = (368, 1168)


def _font(path: Path, size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(path), size)


def _tracked(d: ImageDraw.ImageDraw, xy, text: str, font, fill, track: float,
             anchor_center: bool = True) -> float:
    """자간을 벌려 글자를 찍는다. PIL 에는 자간 기능이 없어 한 글자씩 그린다.

    작은 대문자를 자간 없이 찍으면 뭉쳐 보인다. 실물 라벨의 브랜드명·품목 설명이
    거의 항상 자간을 벌려 놓는 이유다.

    Returns:
        그린 폭 [px].
    """
    widths = [d.textlength(c, font=font) for c in text]
    total = sum(widths) + track * (len(text) - 1)
    x, y = xy
    if anchor_center:
        x -= total / 2
    for c, w in zip(text, widths):
        d.text((x, y), c, font=font, fill=fill)
        x += w + track
    return total


def _wrapped(fn) -> None:
    """이음매를 넘는 요소를 좌우 양쪽에 그린다.

    캔버스 밖으로 나간 부분은 PIL 이 알아서 잘라내므로, 세 번 그려도 안쪽 요소는
    한 번만 남는다.
    """
    for dx in (-LABEL_W, 0, LABEL_W):
        fn(dx)


# ── 무늬 ────────────────────────────────────────────────────────────────
def _field_pattern(img: Image.Image, d: ImageDraw.ImageDraw, design) -> None:
    """바탕에 아주 옅은 사선 격자를 깐다.

    단색으로 두면 플라스틱처럼 보인다. 바탕색보다 한 단계만 밝은 선을 넣으면
    인쇄물의 결이 생기면서도 글자를 방해하지 않는다.
    """
    f = design.field_rgb
    tint = tuple(min(255, int(c + 16)) for c in f)
    step = 26
    for k in range(-LABEL_H // step, (LABEL_W + LABEL_H) // step + 1):
        x = k * step
        d.line([(x, 0), (x + LABEL_H, LABEL_H)], fill=tint, width=1)


def _rules(d: ImageDraw.ImageDraw, design) -> None:
    """위아래 테두리 괘선."""
    a, ink = design.accent_rgb, design.ink_rgb
    d.rectangle([0, 0, LABEL_W, EDGE], fill=a)
    d.rectangle([0, LABEL_H - EDGE - 1, LABEL_W, LABEL_H], fill=a)
    d.rectangle([0, EDGE + 3, LABEL_W, EDGE + 3 + INNER], fill=ink)
    d.rectangle([0, LABEL_H - EDGE - 4 - INNER, LABEL_W, LABEL_H - EDGE - 4], fill=ink)


# ── 그림 ────────────────────────────────────────────────────────────────
def _roundel(d: ImageDraw.ImageDraw, cx: int, cy: int, r: int, design) -> None:
    """그림을 담는 원형 판. 크림 바탕에 금색 두 겹 링."""
    a, ink = design.accent_rgb, design.ink_rgb
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=a)
    d.ellipse([cx - r + 5, cy - r + 5, cx + r - 5, cy + r - 5], fill=ink)
    d.ellipse([cx - r + 12, cy - r + 12, cx + r - 12, cy + r - 12],
              outline=a, width=2)


def emblem_sardine(d: ImageDraw.ImageDraw, cx: int, cy: int, r: int, design) -> None:
    """정어리 옆모습.

    정어리는 **길고 가는** 방추형이다. 짧고 두껍게 그리면 붕어처럼 보여서
    "기름에 절인 정어리" 라는 품목과 어긋난다. 길이:높이를 4:1 가까이 잡는다.
    """
    body = design.field_rgb
    belly = tuple(min(255, c + 78) for c in design.field_rgb)
    L, H = r * 1.96, r * 0.25
    x0 = cx - L / 2
    span = L * 0.78                    # 몸통 구간. 나머지는 꼬리가 쓴다

    def half(t: float) -> float:
        # t=0 코끝, t=1 꼬리자루. 앞쪽 1/3 에서 최대가 되도록 지수를 잡고,
        # 그 최대값이 정확히 H 가 되게 정규화한다.
        p, q = 0.42, 0.62
        peak = (p / (p + q)) ** p * (q / (p + q)) ** q
        return H * (t ** p) * ((1 - t) ** q) / peak

    steps = 48
    top = [(x0 + span * (i / steps), cy - half(i / steps)) for i in range(steps + 1)]
    bot = [(x0 + span * (i / steps), cy + half(i / steps) * 0.92) for i in range(steps + 1)]
    d.polygon(top + bot[::-1], fill=body)

    # 배 쪽 은색 — 아래 절반만 밝게. 정어리의 가장 알아보기 쉬운 특징이다.
    mid = [(x, cy + (y - cy) * 0.34) for x, y in bot]
    d.polygon(bot + mid[::-1], fill=belly)

    # 꼬리 — 자루 끝에서 갈라지는 부채꼴
    px = x0 + span
    d.polygon([(px - 2, cy - H * 0.26), (px - 2, cy + H * 0.26),
               (px + L * 0.20, cy + H * 1.35), (px + L * 0.09, cy),
               (px + L * 0.20, cy - H * 1.35)], fill=body)
    # 등지느러미 — 몸통 중간에서 뒤로 눕는다
    d.polygon([(cx - L * 0.06, cy - half(0.44) + 1), (cx + L * 0.16, cy - half(0.72) + 1),
               (cx + L * 0.02, cy - half(0.44) - H * 0.95)], fill=body)
    # 가슴지느러미
    d.polygon([(x0 + span * 0.26, cy + half(0.26) * 0.5),
               (x0 + span * 0.44, cy + half(0.40) * 0.6),
               (x0 + span * 0.28, cy + half(0.26) * 0.95 + H * 0.5)], fill=body)
    # 아가미뚜껑과 눈
    gx = x0 + span * 0.17
    d.line([(gx, cy - half(0.17) * 0.92), (gx - 3, cy + half(0.17) * 0.80)],
           fill=belly, width=2)
    ex = x0 + span * 0.075
    d.ellipse([ex - 6, cy - 7, ex + 6, cy + 5], fill=design.ink_rgb)
    d.ellipse([ex - 3, cy - 4, ex + 2, cy + 1], fill=body)


def emblem_fig(d: ImageDraw.ImageDraw, cx: int, cy: int, r: int, design) -> None:
    """반으로 자른 무화과.

    무화과는 **위가 좁고 아래가 불룩한** 물방울꼴이다. 위아래 대칭으로 그리면
    올리브나 자두가 되어 버린다. 그래서 폭 함수를 위아래 비대칭으로 잡고, 잘린
    단면의 속살과 씨까지 그려야 무화과로 읽힌다.
    """
    skin = design.field_rgb
    flesh = (198, 96, 118)          # 속살 — 바탕(보라)과도 잉크(연분홍)와도 갈린다
    gold = design.accent_rgb
    W, Hh = r * 0.66, r * 0.58
    by = cy + r * 0.16              # 몸통 중심. 위에 꼭지와 잎이 들어갈 자리를 남긴다

    def width(s: float) -> float:
        # s=0 꼭지쪽, s=1 바닥. 위는 좁게(지수 1.1), 아래는 둥글게 닫는다.
        # 바닥 항의 지수를 크게 잡으면(s**7) 마지막에 급히 닫혀 밑면이 평평한
        # 삼각형이 된다 — 무화과가 아니라 고깔이 보인다. s**4 로 완만하게 닫는다.
        return ((1 - (1 - s) ** 1.1) ** 0.9) * ((1 - s ** 4) ** 0.5)

    peak = max(width(i / 200) for i in range(201))

    def outline(scale: float):
        steps, pts = 44, []
        for i in list(range(steps + 1)) + list(range(steps, -1, -1)):
            s = i / steps
            w = W * scale * width(s) / peak
            x = cx + (w if len(pts) <= steps else -w)
            pts.append((x, by - Hh * scale + 2 * Hh * scale * s))
        return pts

    d.polygon(outline(1.0), fill=skin)
    d.polygon(outline(0.72), fill=flesh)
    # 씨 — 속살 안쪽에만. 시드를 고정해 다시 만들어도 같은 그림이 나온다.
    rng = random.Random(7)
    for _ in range(52):
        s = rng.uniform(0.18, 0.93)
        w = W * 0.72 * width(s) / peak
        x = cx + rng.uniform(-w * 0.74, w * 0.74)
        y = by - Hh * 0.72 + 2 * Hh * 0.72 * s
        d.ellipse([x - 2, y - 1, x + 2, y + 2], fill=gold)

    # 꼭지
    ty = by - Hh
    d.polygon([(cx - 4, ty + 3), (cx + 4, ty + 3),
               (cx + 3, ty - r * 0.24), (cx - 3, ty - r * 0.24)], fill=gold)
    # 잎 — 꼭지 오른쪽으로 뻗는 뾰족한 타원. 가운데 잎맥을 넣어야 잎으로 보인다.
    lx, ly = cx + 2, ty - r * 0.20
    leaf, ln = [], 16
    for i in range(ln + 1):                       # 위 가장자리 (뿌리 → 끝)
        u = i / ln
        leaf.append((lx + r * 0.74 * u,
                     ly - r * 0.20 * math.sin(math.pi * u) * (1 - 0.35 * u)))
    for i in range(ln, -1, -1):                   # 아래 가장자리 (끝 → 뿌리)
        u = i / ln
        leaf.append((lx + r * 0.74 * u,
                     ly + r * 0.17 * math.sin(math.pi * u) * (1 - 0.25 * u)))
    d.polygon(leaf, fill=gold)
    d.line([(lx + 4, ly), (lx + r * 0.68, ly - r * 0.03)], fill=skin, width=2)


def emblem_corn(d: ImageDraw.ImageDraw, cx: int, cy: int, r: int, design) -> None:
    """옥수수 속대 — 낟알 격자와 밑동의 껍질 잎.

    옥수수는 **위로 갈수록 좁아지는 낟알 기둥**이다. 낟알을 격자로 심되 속대
    윤곽 안에만 두고, 밑동에서 껍질 잎이 양옆으로 벌어져야 옥수수로 읽힌다.
    잎 없이 낟알 기둥만 그리면 파인애플이나 벌집으로 보인다.
    """
    kernel = design.field_rgb                       # 낟알 — 라벨 바탕과 같은 노랑
    deep = tuple(max(0, c - 72) for c in kernel)    # 낟알 사이 골
    husk = design.accent_rgb
    husk_lit = tuple(min(255, c + 42) for c in husk)

    cob_h = r * 1.42
    top = cy - cob_h * 0.60                         # 아래를 조금 남겨 잎이 들어간다

    def half_w(t: float) -> float:
        # t=0 위끝(둥근 꼭지), t=1 밑동. 위쪽 1/4 에서 빠르게 넓어지고
        # 아래는 거의 곧다 — 속대의 실루엣이다.
        return r * 0.40 * (1 - (1 - t) ** 2.2) ** 0.5

    steps = 36
    right = [(cx + half_w(i / steps), top + cob_h * (i / steps))
             for i in range(steps + 1)]
    left = [(cx - half_w(i / steps), top + cob_h * (i / steps))
            for i in range(steps, -1, -1)]
    d.polygon(right + left, fill=deep)

    # 낟알 — 줄줄이 심는다. 줄마다 반 칸씩 어긋나게 두면 실제 옥수수의
    # 지그재그 배열이 된다. 골(deep)이 낟알 사이로 비쳐 격자로 읽힌다.
    rows = 10
    for i in range(rows):
        t = 0.09 + 0.86 * i / (rows - 1)
        y = top + cob_h * t
        w = half_w(t) * 0.88
        rw = r * 0.062
        n = max(1, int(w / (rw * 1.25)))
        off = (rw * 1.1) if i % 2 else 0.0
        for k in range(-n, n + 1):
            x = cx + off + k * (w * 2) / (2 * n + 1)
            if abs(x - cx) > w:
                continue
            d.ellipse([x - rw, y - rw * 1.15, x + rw, y + rw * 1.15], fill=kernel)

    # 껍질 잎 — 밑동에서 양옆 위로 벌어지는 두 장 + 아래로 처지는 짧은 한 장.
    by = top + cob_h
    for sgn, scale, tone in ((-1, 1.0, husk), (1, 1.0, husk), (0, 0.62, husk_lit)):
        ln, leaf = 16, []
        tip_x = cx + sgn * r * 0.78 * scale
        tip_y = by - r * 0.66 * scale if sgn else by + r * 0.34
        for i in range(ln + 1):                     # 바깥 가장자리 (밑동 → 끝)
            u = i / ln
            bulge = r * 0.30 * scale * math.sin(math.pi * u)
            leaf.append((cx + (tip_x - cx) * u - sgn * bulge * 0.2,
                         by + (tip_y - by) * u + (bulge if sgn == 0 else -bulge * 0.4)))
        for i in range(ln, -1, -1):                 # 안쪽 가장자리 (끝 → 밑동)
            u = i / ln
            bulge = r * 0.16 * scale * math.sin(math.pi * u)
            leaf.append((cx + (tip_x - cx) * u + sgn * bulge,
                         by + (tip_y - by) * u + (bulge * 0.5 if sgn == 0 else bulge)))
        d.polygon(leaf, fill=tone)
        # 잎맥 — 밑동에서 끝까지 한 줄. 이것이 없으면 잎이 아니라 뿔로 보인다.
        d.line([(cx, by - 2), (tip_x, tip_y)], fill=design.ink_rgb, width=2)


EMBLEMS = {"sardine": emblem_sardine, "fig": emblem_fig, "corn": emblem_corn}


# ── 뒷면 ────────────────────────────────────────────────────────────────
def _back_panel(d: ImageDraw.ImageDraw, design, dx: int) -> None:
    """이음매를 가로지르는 뒷면 — 중량 표기와 바코드.

    앞면을 이음매에서 떼어 놓으려면 그 반대편에 뭔가는 있어야 한다. 비워 두면
    캔의 절반이 민무늬가 되어 오히려 눈에 띈다.
    """
    cx = dx                                   # 이음매 = 0 (그리고 W)
    ink, a = design.ink_rgb, design.accent_rgb

    d.line([(cx - 168, 40), (cx - 168, LABEL_H - 40)], fill=a, width=2)
    d.line([(cx + 168, 40), (cx + 168, LABEL_H - 40)], fill=a, width=2)

    _tracked(d, (cx, 44), design.net, _font(SANS_C, 17), ink, 2.2)

    # 바코드 — 크림 바탕에 잉크 막대. 굵기를 고정 시드로 뽑아 매번 같게 만든다.
    bx0, bx1, by0, by1 = cx - 96, cx + 96, 86, 152
    d.rectangle([bx0 - 8, by0 - 6, bx1 + 8, by1 + 6], fill=ink)
    rng = random.Random(design.name)
    x = bx0
    while x < bx1 - 2:
        w = rng.choice((2, 2, 3, 5))
        if x + w > bx1:
            break
        d.rectangle([x, by0, x + w - 1, by1], fill=design.field_rgb)
        x += w + rng.choice((2, 3, 4))
    _tracked(d, (cx, by1 + 12), "8 412300 097", _font(SANS_C, 15), ink, 1.6)

    _tracked(d, (cx, 200), "CONSERVAS · LOTE 2411", _font(SANS_C, 15), a, 2.4)


def _side_ornament(d: ImageDraw.ImageDraw, cx: int, design) -> None:
    """앞면과 뒷면 사이의 빈 구간을 채우는 마름모 기둥.

    비워 두면 캔을 비스듬히 볼 때 민무늬 면만 보인다. 캔은 벨트 위에서 어느
    방향으로든 놓이므로, 어느 각도에서 봐도 인쇄물처럼 보여야 한다.
    """
    a = design.accent_rgb
    for k, y in enumerate((LABEL_H // 2 - 54, LABEL_H // 2, LABEL_H // 2 + 54)):
        s = 9 if k == 1 else 6
        d.polygon([(cx, y - s), (cx + s, y), (cx, y + s), (cx - s, y)], fill=a)
    d.line([(cx, 54), (cx, LABEL_H // 2 - 70)], fill=a, width=1)
    d.line([(cx, LABEL_H // 2 + 70), (cx, LABEL_H - 54)], fill=a, width=1)


# ── 조립 ────────────────────────────────────────────────────────────────
def render(design) -> Image.Image:
    img = Image.new("RGB", (LABEL_W, LABEL_H), design.field_rgb)
    d = ImageDraw.Draw(img)
    _field_pattern(img, d, design)
    _rules(d, design)

    ink, a = design.ink_rgb, design.accent_rgb

    # 앞면 — 왼쪽에 그림 원판, 오른쪽에 글자 블록. 실물 통조림 라벨의 기본 구도다.
    r = 84
    ecx, ecy = FRONT[0] + 128, LABEL_H // 2
    _roundel(d, ecx, ecy, r, design)
    EMBLEMS[design.emblem](d, ecx, ecy, r - 22, design)

    tx = (FRONT[0] + 268 + FRONT[1]) // 2       # 글자 블록의 중심
    _tracked(d, (tx, 44), design.brand, _font(SANS, 24), a, 7.0)
    d.line([(FRONT[0] + 268, 84), (FRONT[1], 84)], fill=a, width=2)
    _tracked(d, (tx, 96), design.product, _font(SERIF, 62), ink, 4.0)
    d.line([(tx - 92, 178), (tx + 92, 178)], fill=a, width=1)
    _tracked(d, (tx, 190), design.tagline, _font(SANS_C, 20), ink, 4.0)

    _side_ornament(d, (168 + FRONT[0]) // 2, design)
    _side_ornament(d, (FRONT[1] + LABEL_W - 168) // 2, design)
    _wrapped(lambda dx: _back_panel(d, design, dx))

    # 앞면 도안이 캔의 **+x 쪽**을 보게 반 바퀴 돌린다.
    #
    # u=0 이 캔의 로컬 +x 이고, 정면 카메라(camera.py 의 TeleopFrontCameraCfg)가
    # x=1.65 에서 벨트를 마주 본다. 회수 텔레포트가 캔 자세를 항상 (1,0,0,0) 으로
    # 되돌리므로, 벨트를 타고 오는 캔은 예외 없이 +x 면을 정면 카메라에 보인다.
    # 돌리지 않으면 그 자리에 뒷면(바코드)이 와서, 데이터셋 영상에 품목명이 한 번도
    # 안 잡힌다 — 실제로 처음 렌더에서 그렇게 나왔다.
    #
    # 이미지를 통째로 미는 것이라 이음매는 그대로 맞는다. 원통 지도를 회전시키는
    # 것과 같아서, 잘리는 것은 없고 어디가 이음매가 되는지만 바뀐다.
    half = LABEL_W // 2
    rolled = Image.new("RGB", (LABEL_W, LABEL_H))
    rolled.paste(img.crop((half, 0, LABEL_W, LABEL_H)), (0, 0))
    rolled.paste(img.crop((0, 0, half, LABEL_H)), (LABEL_W - half, 0))
    return rolled


def main() -> None:
    out = Path(__file__).resolve().parents[1] / "env/asset/objects/cans/textures"
    out.mkdir(parents=True, exist_ok=True)
    for design in DESIGNS:
        path = out / design.texture_name
        render(design).save(path, optimize=True)
        print(f"생성: {path.relative_to(path.parents[4])}  "
              f"{LABEL_W}x{LABEL_H}  {path.stat().st_size // 1024}KB")


if __name__ == "__main__":
    main()
