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

**모든 캔이 같은 파랑**이다 (2026-08-14 사용자 지시). 예전에는 빨강·노랑·
파랑·보라로 갈라 두었는데, 색이 갈리면 정책이 캔을 **색으로** 식별할 여지가
생긴다. 특히 평가 환경의 과제는 "정상품만 담기" 라 판단 근거가 **형상**(부푼
뚜껑·찌그러진 옆면)이어야 하는데, 색이 단서로 남으면 그 일반화를 확인할 수
없다. 색을 통일하면 남는 차이는 라벨 문구·엠블럼과 형상뿐이다.

바탕은 짙은 마린 블루 (14, 59, 92) 하나로 맞추고, 잉크는 그 위에서 읽히는
따뜻한 크림, 괘선은 놋쇠로 통일한다 — 원래 sardine_can 조합이라 가독성이
이미 확인된 값이다.
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

# normal_can 은 이 표에 넣지 않는다 — 라벨 그림 없이 단색 재질만 쓰는 대조군이고,
# 여기 넣으면 텍스처가 붙어 성격이 바뀐다. 단색도 같은 파랑이다(make_cans.py).
DESIGNS = [
    # normal_can — 예전에는 라벨 그림 없이 단색 재질만 쓰는 대조군이었다. 그런데
    # 다른 캔이 전부 같은 파랑 라벨을 갖게 되자 **혼자만 민무늬 남색**이라
    # 눈에 띄었다(2026-08-14 사용자 지적). 대조군의 원래 목적은 "정상품과
    # 파열품이 같은 도안을 쓴다" 였지 "라벨이 없다" 가 아니므로, 다른 캔과 같은
    # 형식의 라벨을 준다. 짝인 파열품 이름만 예외적으로 burst_can 이다
    # (규칙대로면 normal_can_burst — make_cans.py 가 이 하나를 특별히 다룬다).
    CanDesign(
        name="normal_can",
        product="JUDÍAS BLANCAS",
        brand="PUERTO NUEVO",
        tagline="COCIDAS AL NATURAL",
        net="PESO NETO 400 g",
        field_rgb=(14, 59, 92),        # 마린 블루 — 전 캔 공통
        ink_rgb=(242, 229, 200),       # 따뜻한 크림 — 전 캔 공통
        accent_rgb=(201, 146, 47),     # 놋쇠 — 전 캔 공통
        emblem="bean",
    ),
    CanDesign(
        name="sardine_can",
        product="SARDINAS",
        brand="COSTA AZUL",
        tagline="EN ACEITE DE OLIVA",
        net="PESO NETO 106 g",
        field_rgb=(14, 59, 92),        # 마린 블루 — 전 캔 공통
        ink_rgb=(242, 229, 200),       # 따뜻한 크림 — 전 캔 공통
        accent_rgb=(201, 146, 47),     # 놋쇠 — 전 캔 공통
        emblem="sardine",
    ),
    CanDesign(
        name="fig_can",
        product="HIGOS",
        brand="VALLE DORADO",
        tagline="EN ALMIBAR LIGERO",
        net="PESO NETO 420 g",
        field_rgb=(14, 59, 92),        # 마린 블루 — 전 캔 공통
        ink_rgb=(242, 229, 200),       # 따뜻한 크림 — 전 캔 공통
        accent_rgb=(201, 146, 47),     # 놋쇠 — 전 캔 공통
        emblem="fig",
    ),
    # corn_can — 원래는 외부 HOPE 에셋이었는데, 그 메시·물성이 생성 캔들과 달라
    # 파지 때 그리퍼 폭주를 일으켰다(2026-08-10 수집 실측). 그래서 생성 캔으로
    # 교체했다. 색은 2026-08-14 부터 전 캔 공통 마린 블루다.
    CanDesign(
        name="corn_can",
        product="MAÍZ DULCE",
        brand="SOL DE CAMPO",
        tagline="GRANOS ENTEROS TIERNOS",
        net="PESO NETO 340 g",
        field_rgb=(14, 59, 92),        # 마린 블루 — 전 캔 공통
        ink_rgb=(242, 229, 200),       # 따뜻한 크림 — 전 캔 공통
        accent_rgb=(201, 146, 47),     # 놋쇠 — 전 캔 공통
        emblem="corn",
    ),
]

BY_NAME = {d.name: d for d in DESIGNS}
