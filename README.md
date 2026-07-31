# franka_robolab_sim

[NVlabs/RoboLab](https://github.com/NVlabs/RoboLab) 위에 올린 **창고 안 흰색 테이블
pick-and-place 시뮬레이션 + 브라우저 키보드 텔레오퍼레이션**.

브라우저에서 키보드로 Franka 의 End-Effector 를 움직이고 그리퍼를 여닫는다. 마우스
드래그로 시점을 돌리고 휠로 줌한다. Isaac Sim 은 헤드리스로 돌고, 카메라 렌더는
MJPEG 으로 브라우저에 흘려보낸다.

```
브라우저 ──키/마우스──▶ WebSocket ──▶ TeleopState ──[dx,dy,dz,dR,dP,dY,grip]──▶ env.step()
        ◀────MJPEG──── HTTP 스트림 ◀── JPEG 인코딩 ◀── 카메라 렌더 ◀──────────┘
                       (전부 단일 포트 8003)
```

## 왜 MoveIt 을 쓰지 않는가

RoboLab 의 `DroidRelIKActionCfg` 가 Isaac Lab 의 `DifferentialInverseKinematicsActionCfg`
(DLS 기반 `DifferentialIKController`)를 그대로 감싸고 있다. 즉 **IK 는 시뮬레이터 안에서
이미 풀린다.** 액션 벡터에 EEF 델타를 넣으면 관절각은 알아서 나온다.

RoboLab 자체에는 텔레오퍼레이션 코드가 없고, Isaac Lab 의 `Se3Keyboard` 는 `carb.input`
(Omniverse 네이티브 입력)을 구독해서 **GUI 창 포커스를 요구**하므로 원격 브라우저에서는
쓸 수 없다. 그래서 필요한 건 IK 구현이 아니라 브라우저 입력을 액션 벡터로 옮기는
얇은 브리지뿐이고, 그게 `franka_teleop/` 이다.

## 씬

| 항목 | 값 |
|---|---|
| 작업대 | 흰색 테이블, **좌우(Y) 160cm × 앞뒤(X) 150cm**, 상판 z=0 |
| 배경 | Isaac Sim 기본 창고 `Simple_Warehouse/warehouse.usd` (24m × 38.8m, 바닥 z=−0.70) |
| 로봇 | RoboLab Droid 구성 (Franka 팔 + Robotiq 2F-85), 베이스가 원점 |
| 물체 | YCB 바나나 + 그릇. 성공 판정 = 바나나가 그릇 안 |

치수는 손으로 계산한 값이 아니라 실제 로드된 스테이지에서 잰 값이다. 기동 로그의
`[teleop] 테이블 상판:` / `[teleop] 창고:` 줄에 매번 찍힌다
(`run_teleop.py:log_scene_bounds`).

## 구성

| 경로 | 역할 |
|---|---|
| `run_teleop.py` | 엔트리포인트 — 환경 등록, 메인 루프, 카메라 갱신, 프레임/텔레메트리 발행 |
| `franka_teleop/config.py` | 키맵, 이동 스케일, 카메라 궤도, 안전박스, 포트 |
| `franka_teleop/state.py` | 심 스레드 ↔ 웹 스레드 공유 상태 (클라이언트별 입력 관리) |
| `franka_teleop/web_server.py` | aiohttp — HTML / MJPEG / WebSocket 을 8003 한 포트에서 |
| `franka_teleop/camera.py` | 텔레오퍼레이션 전용 3인칭 시점 |
| `franka_teleop/background.py` | Isaac Sim 기본 창고 배경 |
| `franka_teleop/safety.py` | EEF 목표 안전박스·반경 클램프 |
| `franka_teleop/static/index.html` | 조작 UI |
| `tasks/white_table_pick_place_task.py` | 태스크 정의 |
| `assets/scenes/white_table_pick_place.usda` | 흰색 테이블 씬 |
| `tools/test_teleop.py` | 브라우저 없이 도는 스모크 테스트 |
| `scripts/` | 빌드 / 컨테이너 / 기동·정지 스크립트 |

## 요구사항

- NVIDIA GPU + `nvidia-container-toolkit`
- `~/robolab` 에 RoboLab 저장소 (git-lfs 로 에셋까지 받은 상태)
- 디스크 ~50GB, 인터넷 (창고 에셋을 NVIDIA 클라우드에서 받아 캐시한다)

## 사용법

```bash
./scripts/build.sh          # 이미지 빌드 (robolab:teleop 베이스가 없으면 같이 만든다)
./scripts/container_up.sh   # 컨테이너 기동
./scripts/teleop_start.sh   # 텔레오퍼레이션 시작
```

브라우저에서 `http://<서버주소>:8003` 접속. 창을 한 번 클릭해 포커스를 준 뒤 키를 누른다.

정지는 `./scripts/teleop_stop.sh`, 로그는 `./scripts/logs.sh`.

동작 검증(브라우저 없이): `python3 tools/test_teleop.py --host <서버주소>`

## 조작

| 키 | 동작 | 키 | 동작 |
|---|---|---|---|
| `W` / `S` | 화면 안 / 앞 | `Z` / `X` | roll ± |
| `A` / `D` | 화면 좌 / 우 | `T` / `G` | pitch ± |
| `Q` / `E` | 위 / 아래 | `C` / `V` | yaw ± |
| `Space` | 그리퍼 열기/닫기 | `R` | 에피소드 리셋 |
| `F` | 시점 초기화 | `[` / `]` | 배속 −/+ (0.25×~2×) |

마우스 **드래그**로 시점 회전, **휠**로 줌.

이동 키는 월드축이 아니라 **화면 기준**이다. 시점을 돌리면 W 가 가리키는 월드 방향도
같이 돌아가므로, 카메라를 어디로 돌리든 "W 는 화면 안쪽" 이 유지된다
(`run_teleop.py:screen_to_world`). 회전 키(roll/pitch/yaw)만 월드 고정이다.

## 알아둘 것 (여기서 시간을 많이 썼다)

### RobolabEnv 는 리셋하지 않고 "freeze" 한다

`RobolabEnv` 는 정책 벤치마크용이라, 에피소드가 한 번이라도 스텝된 뒤 종료되면
env 를 리셋하는 대신 **freeze** 시킨다. freeze 된 env 는 이렇게 동작한다.

```python
# robolab/core/environments/env.py
def step(self, action):
    if self._frozen_envs.any():
        action = action.clone()
        action[self._frozen_envs] = 0.0     # 액션이 통째로 0 이 된다
    return super().step(action)
```

그래서 `env.reset()` 만 부르면 **겉보기엔 팔이 홈 자세로 돌아가지만 그 뒤로 키 입력이
전혀 먹지 않는다.** 관측값도 종료 시점에 멈춘 채로 남아 UI 좌표가 얼어붙는다.
증상이 "리셋은 된 것 같은데 조작이 안 된다" 라서 원인을 찾기 어렵다.

해결은 `reset_eval_state()` 로 freeze 플래그와 `_has_stepped` 를 내리는 것이다
(`run_teleop.py:reset_episode`).

```python
end_episode(env)          # 레코더 정리 — 안 부르면 매 스텝 기록이 무한히 쌓인다
env.reset_eval_state()    # freeze 해제 (이게 핵심)
obs, _ = env.reset()
```

### 입력 타임아웃은 클라이언트별이어야 한다

처음에는 "마지막 입력 시각" 을 전역으로 하나만 뒀는데, 그러면 **다른 탭이 보내는
하트비트가 조작 중인 클라이언트의 타임아웃을 갱신해버린다.** 조작하던 브라우저가
죽어도 구경만 하는 탭이 하나 열려 있으면 로봇이 계속 움직인다. 지금은 연결마다
`_Client` 를 두고 눌림 키와 하트비트를 따로 관리한다 (`franka_teleop/state.py`).

### 태스크의 contact_object_list 에 "table" 이 필요하다

`pick_and_place` 서브태스크가 내부적으로 `gripper_hit_table` 프레디킷을 쓰기 때문에
`gripper__table` 접촉 센서가 있어야 한다. 빠지면 첫 스텝에서 `ValueError` 로 죽는다.
그래서 씬의 상판 프림 이름도 반드시 `table` 이어야 한다.

### 상대 IK 는 명령한 델타를 그대로 따라가지 않는다

DLS 감쇠와 PD 추종 지연 때문에 대략 1/3 수준만 반영된다. `config.POS_DELTA` 는 그 점을
감안해 실측으로 잡은 값이다 (현재 1× 에서 약 4.4cm/s, 2× 에서 약 9cm/s).

### RoboLab HeadCameraCfg 는 화면이 90° 돌아간다

쿼터니언 성분 순서가 어긋나 있다. `franka_teleop/camera.py` 는 look-at 을 직접 계산해
쓴다 — 시점을 바꾸려면 성분을 눈대중으로 만지지 말고 look-at 을 다시 계산할 것.

### 창고 바닥은 z=0 이 아니다

`warehouse.usd` 는 자기 바닥이 z=0 이지만 이 프로젝트는 테이블 상판이 z=0, 지면이
z=−0.697 이다. `background.py` 에서 창고를 그만큼 내려 바닥을 맞춘다. 씬 USDA 쪽에
보이는 바닥을 또 깔면 창고 바닥과 z-fighting 이 나므로 충돌면만 남겼다.

## 안전장치

원격 조작이라 입력이 끊기면 로봇이 폭주할 수 있다. 세 겹으로 막는다.

1. **작업공간 클램프** — 매 스텝 "현재 EEF + 델타"를 안전박스(`SAFE_BOX_LO/HI`)와
   반경(`SAFE_RADIUS`)으로 자른다. 상대 IK 는 델타를 적분하므로 이게 없으면 발산한다.
2. **입력 타임아웃** — 브라우저가 200ms 마다 하트비트를 보낸다. `INPUT_TIMEOUT_S`(0.5초)
   동안 아무것도 안 오면 그 클라이언트의 키를 전부 뗀다.
3. **포커스 이탈 감지** — 탭이 포커스를 잃으면 `keyup` 이 오지 않으므로, `blur` 이벤트에서
   즉시 전체 키를 해제한다. WebSocket 이 끊길 때도 그 클라이언트의 키가 같이 사라진다.

램프(`RAMP_STEPS`)로 키를 누른 직후 델타를 서서히 올려 IK 점프도 줄인다.

## 마운트 배치 주의

`assets/scenes/white_table_pick_place.usda` 의 payload 경로가
`../../../robolab/assets/...` 상대경로다. 따라서 컨테이너 안에서 두 저장소가
**`/workspace` 아래 형제로** 붙어 있어야 한다.

```
/workspace/robolab              ← RoboLab (에셋 제공)
/workspace/franka_robolab_sim   ← 이 저장소
```

`scripts/container_up.sh` 가 이 배치를 보장한다.

## 하드웨어 메모

RoboLab 은 48GB+ VRAM 을 권장하지만 이 설정은 RTX 3090(24GB) 한 장을 전제로 한다.
`num_envs=1`, 부가 센서 제거, 뷰포트 카메라 1대만 남긴 상태에서 **실측 9.5GB** 로
여유 있게 들어간다. 제어율은 약 7Hz — 렌더링이 병목이다. 카메라를 늘리거나
해상도를 올리면 둘 다 다시 확인해야 한다.
