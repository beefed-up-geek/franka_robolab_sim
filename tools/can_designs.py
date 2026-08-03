#!/usr/bin/env python3
"""캔 라벨 도안 표 — 생성기 두 개가 공유하는 **단일 출처**.

    tools/make_can_labels.py   이 표를 보고 라벨 이미지를 그린다 (PIL 필요)
    tools/make_cans.py         이 표를 보고 캔 USD 를 뽑는다 (의존성 없음)

둘로 나눈 이유는 형상 생성기를 의존성 없이 두기 위해서다. 캔 모양은 순수 파이썬
으로 만들 수 있는데 라벨 그림은 PIL 이 있어야 하니, PIL 이 없는 곳에서도 형상은
다시 만들 수 있어야 한다. 그래서 이 파일에는 **데이터만** 둔다 — import 해도
아무것도 그리지 않는다.

이름·색·문구가 두 곳에 흩어지면 한쪽만 고쳐져 라벨 그림과 USD 가 어긋난다.
파일 이름은 여기서 정하는 `texture` 하나로 통일된다.

## 색을 고르는 기준

정책이 캔을 구분해야 하므로 **색상환에서 서로 멀어야** 한다. 이미 붉은색
(normal_can 의 단색 라벨)과 노란색(hope/corn_can 의 옥수수 사진)이 있으므로,
새로 넣는 둘은 파랑과 보라로 잡았다. 채도·명도가 아니라 **색상**이 갈려야
저해상도 손목 카메라에서도 구분된다.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class CanDesign:
    """캔 한 종류. 정상품과 파열품이 이 도안을 **함께** 쓴다.

    같은 도안을 쓰는 것이 핵심이다. 파열품만 라벨이 다르면 정책이 결함이 아니라
    라벨 그림을 외워서 골라내게 된다 (objects/cans/README.md 참고).
    """
    name: str                 # 정상품 프림 이름. 파열품은 여기에 _burst 가 붙는다
    product: str              # 라벨에 크게 들어가는 품목명
    brand: str                # 상단 워드마크
    tagline: str              # 품목명 아래 한 줄
    net: str                  # 뒷면 소량 표기
    field_rgb: tuple          # 바탕색
    ink_rgb: tuple            # 본문 잉크 (바탕 위에서 읽혀야 한다)
    accent_rgb: tuple         # 괘선·워드마크
    emblem: str               # make_can_labels.py 의 그림 함수 이름
    texture: str = field(default="")

    @property
    def texture_name(self) -> str:
        """라벨 이미지 파일 이름. 지정이 없으면 이름에서 만든다."""
        return self.texture or f"{self.name}_label.png"

    @property
    def burst_name(self) -> str:
        return f"{self.name}_burst"


# 라벨 이미지 크기 [px]. 캔 몸통을 한 바퀴 감으므로 가로가 둘레, 세로가 라벨 띠
# 높이에 대응한다. 실제 비율은 둘레 213.6mm / 띠 높이 37.1mm = 5.76:1 인데
# 6:1 로 그린다 — 4% 세로로 늘어나지만 눈에 띄지 않고, 정수 비가 작업하기 쉽다.
LABEL_W, LABEL_H = 1536, 256

# 새로 넣은 두 종류. normal_can(단색 적)은 이 표에 넣지 않는다 — 그 캔은 라벨
# 그림 없이 단색 재질만 쓰는 대조군이고, 여기 넣으면 텍스처가 붙어 성격이 바뀐다.
DESIGNS = [
    CanDesign(
        name="sardine_can",
        product="SARDINAS",
        brand="COSTA AZUL",
        tagline="EN ACEITE DE OLIVA",
        net="PESO NETO 106 g",
        field_rgb=(14, 59, 92),        # 짙은 마린 블루
        ink_rgb=(242, 229, 200),       # 따뜻한 크림 — 파랑 위에서 잘 읽힌다
        accent_rgb=(201, 146, 47),     # 놋쇠
        emblem="sardine",
    ),
    CanDesign(
        name="fig_can",
        product="HIGOS",
        brand="VALLE DORADO",
        tagline="EN ALMIBAR LIGERO",
        net="PESO NETO 420 g",
        field_rgb=(62, 31, 77),        # 짙은 가지색(보라)
        ink_rgb=(235, 211, 228),       # 연한 분홍빛 회색
        accent_rgb=(217, 164, 65),     # 금
        emblem="fig",
    ),
]

BY_NAME = {d.name: d for d in DESIGNS}
