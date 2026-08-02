# env — 시뮬레이션 환경

Isaac Sim 위에서 도는 로봇 환경. 브라우저에서 텔레오퍼레이션으로 조작하고,
`data_collection` 과 `inference` 가 여기에 붙어 데이터를 모으거나 정책을 돌린다.

```
env/
├── asset/    USD 자산 — 컨베이어, 씬, 물체
├── src/      파이썬 소스 — 환경 구성, 컨베이어 구동, 텔레오퍼레이션 서버
└── script/   환경별 실행 스크립트 (실험 환경마다 하나씩)
```

## 실행

```bash
./scripts/sim_start.sh env_test      # 컨베이어 + 블록 (기본 확인용)
./scripts/sim_start.sh env_cans      # 컨베이어 + 통조림 7종 + 회색 통
```

브라우저에서 `http://<서버주소>:8003` 접속. 정지는 `./scripts/sim_stop.sh`.

## asset/

| 경로 | 내용 |
|---|---|
| `asset/conveyor/conveyor.usda` | 소형 벨트 컨베이어 800×200×200mm (도면 FRS-CV-001) |
| `asset/scenes/conveyor_pick_place.usda` | env_test 용 — 작업대 + 블록 + 그릇 |
| `asset/scenes/can_sorting.usda` | env_cans 용 — 작업대 + 통조림 7종 |
| `asset/objects/` | RoboLab 에서 가져온 물체 (참조되는 것만, 텍스처 포함) |
| `asset/fixtures/grey_bin.usd` | 담을 통 420×280×105mm |

씬의 payload 경로는 모두 `asset/` 안을 가리키는 상대경로다. RoboLab 마운트 위치에
기대지 않으므로 컨테이너 배치가 바뀌어도 깨지지 않는다.

## src/

| 경로 | 역할 |
|---|---|
| `src/franka_env/runner.py` | 환경 등록 + 메인 루프. 실행 스크립트가 `run()` 을 부른다 |
| `src/franka_env/cli.py` | 실행 스크립트 공통 인자 (Isaac Sim 기동 **전에** import 되므로 무거운 것 금지) |
| `src/franka_env/config.py` | 키맵, 이동 스케일, 카메라 궤도, 벨트 속도, 안전박스, 포트 |
| `src/franka_env/conveyor.py` | 벨트 구동과 화물 순환 |
| `src/franka_env/world_assets.py` | 창고 배경 + 컨베이어 배치 |
| `src/franka_env/camera.py` | 텔레오퍼레이션 전용 3인칭 시점 |
| `src/franka_env/state.py` | 심 스레드 ↔ 웹 스레드 공유 상태 (클라이언트별 입력 관리) |
| `src/franka_env/web_server.py` | aiohttp — HTML / MJPEG / WebSocket 을 8003 한 포트에서 |
| `src/franka_env/safety.py` | EEF 목표 안전박스·반경 클램프 |
| `src/tasks/` | 태스크 정의 (씬, 종료 조건, 접촉 대상) |

## 환경 목록

| 스크립트 | 화물 | 담는 곳 | 특징 |
|---|---|---|---|
| `env_test` | 45mm 블록 6개 | YCB 그릇 | 크기가 같아 파지가 단순 |
| `env_cans` | 통조림 7종 | grey_bin | 높이가 33/58/83mm 로 달라 파지 높이를 매번 맞춰야 한다 |

`env_cans` 는 벨트에 4개만 올리고 나머지는 상판 아래 대기열에 둔다. 간격(0.18m)을
넓게 유지하면서도 7종이 돌아가며 흐르게 하기 위해서다. 벨트 사용 구간이 0.72m 라
간격과 종류 수는 서로 상충하는데, 대기열이 그 균형을 잡아 준다.

## script/ — 환경을 새로 만들 때

`env_test.py` 를 복사해 태스크와 기본값만 바꾸면 된다. `runner.py` 는 건드리지 않는다.

```python
parser = build_parser(
    description="...",
    task="MyTask",        # env/src/tasks 의 클래스 이름
    camera="behind",
)
```

새 태스크는 `src/tasks/` 에 파일을 추가한다. 그러면
`./scripts/sim_start.sh <스크립트이름>` 으로 바로 돌아간다.

**주의**: 실행 스크립트의 import 순서를 지켜야 한다.
`cv2` → `sys.path` → 인자 파싱 → `AppLauncher` → `franka_env.runner`.
Isaac Sim 은 앱을 띄운 뒤에야 isaaclab/robolab 을 import 할 수 있다.

## 조작

| 키 | 동작 | 키 | 동작 |
|---|---|---|---|
| `W` / `S` | 화면 안 / 앞 | `Z` / `X` | roll ± |
| `A` / `D` | 화면 좌 / 우 | `T` / `G` | pitch ± |
| `Q` / `E` | 위 / 아래 | `C` / `V` | yaw ± |
| `Space` | 그리퍼 열기/닫기 | `R` | 에피소드 리셋 |
| `F` | 시점 초기화 | `[` / `]` | 배속 −/+ |
| `B` | 컨베이어 정지/가동 | `,` / `.` | 벨트 속도 −/+ |

마우스 드래그로 시점 회전, 휠로 줌. 이동 키는 월드축이 아니라 **화면 기준**이라
시점을 돌려도 "W 는 화면 안쪽" 이 유지된다.

## 아직 없는 것 — ROS 인터페이스

`data_collection` 과 `inference` 가 붙으려면 다음이 필요하다. 아직 구현하지 않았다.

- 물체 자세 퍼블리시 (벨트 위 화물, 그릇)
- 그리퍼 자세 퍼블리시
- 그리퍼 자세 명령 구독 (현재는 브라우저 키 입력만)
- 그리퍼 개폐 명령 구독

지금은 이 값들이 `state.publish_telemetry()` 로 WebSocket 에만 나간다.
ROS 노드를 붙일 때 같은 소스를 쓰면 된다.


## 컨베이어 속도

설정은 m/분 으로 한다 (`config.BELT_SPEED_MPM = [0, 2, 4, 6]`). 실물 컨베이어를
분당 미터로 말하는 관례를 따랐고, 내부 계산용 m/s 는 여기서 환산한다.
기본값은 **2 m/분** — 도면 정격(6.6 m/분)의 30% 로, 사람이 여유 있게 집을 수 있다.

시뮬레이션이 실시간의 약 0.7배로 도는 점에 주의. 벽시계로 재면 더 느리게 보이지만
**시뮬레이션 시간 기준으로는 설정값 그대로**다. 데이터를 수집할 때 타임스탬프는
벽시계가 아니라 시뮬레이션 시간을 써야 한다.
