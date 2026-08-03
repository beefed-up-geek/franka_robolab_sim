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
# 실험 환경 — 조건이 스크립트에 박혀 있다
./scripts/task3_train.sh             # 정상품만
./scripts/task3_test.sh              # 불량품 20% 섞임
```

브라우저에서 `http://<서버주소>:8003` 접속. 정지는 `./scripts/sim_stop.sh`.

## asset/

| 경로 | 내용 |
|---|---|
| `asset/conveyor/conveyor.usda` | 소형 벨트 컨베이어 800×200×200mm (도면 FRS-CV-001) |
| `asset/scenes/_can_workcell.usda` | task3 두 환경이 **공유**하는 고정 설비 (작업대·다리·지면) |
| `asset/scenes/task3_train.usda` | 공유 설비 + 정상품 5종 |
| `asset/scenes/task3_test.usda` | 공유 설비 + 정상품 5종 + 파열품 5종 |
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
| `task3_train_pick_and_place_can` | 정상 통조림 5종 | grey_bin | 결함 없는 물건만 — 시연 수집용 |
| `task3_test_pick_and_place_can` | 정상 5종 + 파열품 5종 | grey_bin | 학습 때 못 본 불량품이 섞여 흐른다 |

두 환경 모두 벨트에 3개만 올리고 나머지는 상판 아래 대기열에 둔다. 간격(0.22m)을
넓게 유지하면서도 7종이 돌아가며 흐르게 하기 위해서다. 벨트 사용 구간이 0.72m 라
간격과 종류 수는 서로 상충하는데, 대기열이 그 균형을 잡아 준다.

## 실험 조건은 scripts/ 안의 실행 파일에 있다

`scripts/task3_train.sh` `scripts/task3_test.sh` 가 각 실험 환경의 조건을 박아 두고
`sim_start.sh` 를 부른다. 인자로 매번 넘기면 어떤 조건으로 돌렸는지 기록이 남지
않는다 — **그 파일이 곧 실험 조건의 기록이고**, 조건을 바꾸면 고쳐서 커밋한다.

```bash
./scripts/task3_test.sh              # 박아 둔 조건 그대로
./scripts/task3_test.sh --seed 7     # 한 번만 다르게 (뒤에 준 인자가 이긴다)
```

새 실험 환경을 만들 때는 이 파일 하나를 복사해 값만 바꾼다
(`tools/make_runner_scripts.py` 가 두 파일을 생성한다).

아래는 그 스크립트가 넘기는 인자들이다.

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `--belt-speed` | 2 m/분 | 컨베이어 초기 속도. 기본 단계(0/2/4/6)에 없는 값이면 목록에 끼워 넣어 `,` `.` 키로도 오갈 수 있게 한다 |
| `--defect-ratio` | 0.2 | 투입 화물 중 불량품 비율. 불량품이 없는 환경(train)에서는 무시된다 |
| `--defect-pattern` | `burst` | 이름에 이 문자열이 들어간 화물을 불량품으로 본다 |
| `--spacing` | 0.22 m | 화물 사이 간격. 입구 근처에 이 거리 안으로 화물이 있으면 투입을 미룬다 |
| `--seed` | 0 | 투입 순서 난수 시드. 같은 시드면 같은 순서가 재현된다 |
| `--physics-device` | `cpu` | 환경이 1개뿐이라 CPU 가 GPU 보다 빠르다 (11Hz vs 4.5Hz) |

**비율은 장기 평균으로만 맞는다.** 투입할 때마다 비율대로 동전을 던져 종류를
고르는데, 대기열에 그 종류가 없으면 그냥 맨 앞을 낸다 — 비율을 맞추겠다고 벨트를
비워 두면 조작할 것이 없어지기 때문이다. 실측으로 20% 설정에서 3분간 23% 가 나왔다.

## 통에 담으면 사라진다

담는 통 안으로 들어간 화물은 대기열로 돌아가고, 텔레메트리의 `binned` 가 올라간다.
안 그러면 통이 넘쳐 얼마 못 가 담을 곳이 없어진다. 통의 위치·크기는 상수가 아니라
스테이지에서 직접 재므로 `world_assets.py` 의 배치를 바꿔도 따라간다.

판정은 **통 테두리보다 낮을 때**만 한다. 통 위를 지나가거나 그리퍼가 위쪽에 들고
있는 동안에는 사라지지 않는다. 다만 테두리 아래까지 내린 채로 쥐고 있으면 그
시점에 사라진다.

### task3 — 학습/평가 쌍

`task3_train` 과 `task3_test` 는 **흐르는 물건만 다르고 설비는 완전히 같다.** 작업대·
다리·지면은 `_can_workcell.usda` 하나를 서브레이어로 공유한다. 씬을 복사해 두면
한쪽만 고쳐져 두 환경이 조용히 달라지고, 그러면 평가 결과가 환경 차이 탓인지 정책
탓인지 구분할 수 없게 된다.

파열품은 정상품과 **짝**을 이룬다 — 원본 메시를 변형해 만든 것이라 텍스처·UV 가
같고 부푼 뚜껑·찌그러진 옆면·뜯긴 구멍만 다르다. 불량품만 라벨이 다르면 그림만
외워도 골라낼 수 있기 때문이다. 자세한 것은
[`asset/objects/cans/README.md`](asset/objects/cans/README.md).

파열품은 아래 뚜껑이 볼록해 벨트 위에서 비스듬히 기운다. train 에서 보지 못한
파지 자세라, 평가에서 실제로 어려운 조건이 된다.

## script/ — 환경을 새로 만들 때

`task3_train_pick_and_place_can.py` 를 복사해 태스크와 기본값만 바꾸면 된다.
`runner.py` 는 건드리지 않는다. 실행 조건은 `scripts/task3_*.sh` 를 복사해 만든다.

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
| `F` | 시점 초기화 | `N` | 전체 초기화 |
| `[` / `]` | 배속 −/+ | `B` | 컨베이어 정지/가동 |
| `,` / `.` | 벨트 속도 −/+ | | |

마우스 드래그로 시점 회전, 휠로 줌. 이동 키는 월드축이 아니라 **화면 기준**이라
시점을 돌려도 "W 는 화면 안쪽" 이 유지된다.

## 초기화 — 프로세스를 죽이지 않는다

씬을 다시 로드하는 데 2분이 걸린다(셰이더 캐시가 있어도). 그래서 어떤 상태에서
빠져나오든 **심을 살려 둔 채로** 되돌릴 수 있게 해 두었다. 강도는 세 단계다.

| 강도 | 되돌리는 것 |
|---|---|
| `soft` | 에피소드만 다시 시작. freeze 해제 + `env.reset()`, 대기열 비우기 |
| `hard` | + 로봇 관절 상태를 시뮬레이터에 직접 덮어쓴다 |
| `full` | + 모든 강체를 씬 기본 자세로, 컨베이어 장부(회수·투입 수)를 0 으로 |

`hard` 가 따로 있는 이유는 그리퍼 링키지가 터진 뒤에는 `env.reset()` 이 관절
**목표**만 되돌리고 밀려난 링크의 실제 자세는 남기기 때문이다 — 그 상태로는 같은
자리에서 다시 터진다(실측 8회 연속). `full` 은 거기에 화물까지 되돌려 사실상
프로세스를 다시 띄운 것과 같은 상태를 만든다.

부르는 방법은 네 가지이고 전부 같은 경로로 들어간다.

```bash
./scripts/sim_reset.sh full                                  # 셸
curl -X POST 'http://localhost:8003/reset?level=full'        # HTTP
ros2 topic pub --once /franka/cmd/reset std_msgs/String '{data: full}'
```

브라우저에서는 `R`(soft) / `N`(full). 요청이 겹치면 **강한 쪽이 이긴다.**

완료는 `GET /telemetry` 의 `reset_count` 가 오르는 것으로 확인한다. ROS 쪽은
`/franka/events` 에 `{"type": "reset_done", "level": ..., "source": ...}` 가 뜬다.

수집기는 시작할 때 자동으로 `full` 을 요청한다(`collect.py --reset`, 기본값 `full`).
실행마다 같은 조건에서 시작해야 성공률 비교가 의미를 갖기 때문이다. 직전 상태를
이어서 보려면 `--reset none`.

그리퍼 폭주는 자동으로 리셋되는데, 짧은 간격으로 **되풀이되면 두 번째부터
`full` 로 올린다**. 리셋이 팔만 되돌리고 손가락을 파고든 화물은 그 자리에 남는 탓에
결정론적 정책이 똑같이 접근해 똑같이 터지기 때문이다(실측 13회 연속).

## ROS 인터페이스

시뮬레이션 프로세스 안에서 ROS 2 노드(`franka_sim`)가 뜬다. Isaac Sim 이 ROS 2 를
번들하고 있어 시스템 ROS 설치는 필요 없다 — 자세한 토픽 목록은
`src/franka_env/ros_node.py` 의 모듈 주석에 있다.


## 컨베이어 속도

설정은 m/분 으로 한다 (`config.BELT_SPEED_MPM = [0, 2, 4, 6]`). 실물 컨베이어를
분당 미터로 말하는 관례를 따랐고, 내부 계산용 m/s 는 여기서 환산한다.
기본값은 **2 m/분** — 도면 정격(6.6 m/분)의 30% 로, 사람이 여유 있게 집을 수 있다.

시뮬레이션이 실시간의 약 0.7배로 도는 점에 주의. 벽시계로 재면 더 느리게 보이지만
**시뮬레이션 시간 기준으로는 설정값 그대로**다. 데이터를 수집할 때 타임스탬프는
벽시계가 아니라 시뮬레이션 시간을 써야 한다.
