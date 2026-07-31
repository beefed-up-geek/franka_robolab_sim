# franka_robolab_sim

[NVlabs/RoboLab](https://github.com/NVlabs/RoboLab) 위에 올린 **창고 안 컨베이어
pick-and-place 시뮬레이션 + 브라우저 키보드 텔레오퍼레이션**.

블록이 컨베이어를 타고 끊임없이 흘러오고, 사람이 브라우저에서 Franka 를 조작해
집어 그릇에 담는다.

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
| 컨베이어 | 800 × 200 × 200mm (도면 FRS-CV-001). 길이가 좌우(Y)를 향하도록 배치, 반송면 z=0.20 |
| 배경 | Isaac Sim 기본 창고 `Simple_Warehouse/warehouse.usd` (24m × 38.8m, 바닥 z=−0.70) |
| 로봇 | RoboLab Droid 구성 (Franka 팔 + Robotiq 2F-85), 베이스가 원점 |
| 물체 | 45mm 블록 6개 + 원기둥 1개(벨트 위 순환) + YCB 그릇(담는 곳) |

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
| `franka_teleop/world_assets.py` | 창고 배경 + 컨베이어 배치 |
| `franka_teleop/conveyor.py` | 벨트 구동과 블록 순환 |
| `franka_teleop/safety.py` | EEF 목표 안전박스·반경 클램프 |
| `franka_teleop/static/index.html` | 조작 UI |
| `tasks/conveyor_pick_place_task.py` | 태스크 정의 |
| `assets/conveyor/conveyor.usda` | 컨베이어 에셋 |
| `assets/scenes/conveyor_pick_place.usda` | 씬 |
| `assets/objects/` | RoboLab YCB 에서 가져온 그릇·바나나 (텍스처 포함) |
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
| `B` | 컨베이어 정지/가동 | `,` / `.` | 벨트 속도 −/+ |

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

### 컨베이어는 PhysX 표면 속도로 만들 수 없었다 (공식 방식 포함)

가장 오래 걸린 부분이다. 컨베이어의 정석은 `PhysxSurfaceVelocityAPI`(표면 속도)이고,
공식 `isaacsim.asset.gen.conveyor` 익스텐션도 소스를 보면 똑같이
`RigidBodyAPI` → `CollisionAPI` → `PhysxSurfaceVelocityAPI` 를 적용한다. 그런데 이
구성(Isaac Lab gym 워크플로 + GPU/Fabric)에서는 두 가지가 겹쳐 못 쓴다.

1. 표면 속도를 적용하는 순간 **그 프림의 콜라이더가 무효화된다.** 블록이 벨트를 그대로
   통과해 테이블로 떨어졌다. Isaac Lab 이슈
   [#4561](https://github.com/isaac-sim/IsaacLab/issues/4561) 에 같은 증상이 있다 —
   GUI 에서는 되는데 gym 워크플로에서만 깨진다.
2. 표면 속도는 **CPU 물리 전용**이라 GPU/Fabric 과 양립하지 않는다
   ([discussion #3216](https://github.com/isaac-sim/IsaacLab/discussions/3216)).

증상이 "콜라이더가 없는 것처럼 보이는" 형태라, 한동안 USD 합성(payload/reference,
정적 vs 키네마틱)을 의심하며 헤맸다. 씬 파일에 직접 넣은 검증용 콜라이더는 잘
동작했는데 — 그 프림에만 표면 속도를 걸지 않았기 때문이었다.

NVIDIA 공식 테스트(`isaacsim.asset.gen.conveyor/tests/test_conveyor.py`)가 요구하는
조건을 전부 맞춘 뒤에도 실패했다 — PhysX 씬이 `gpu_dynamics=False`,
`broadphase=MBP`, `solver=TGS` 이고 벨트가 키네마틱 강체인 상태에서도 물체가 그대로
통과한다. 시도한 조합과 결과:

| 조합 | 결과 |
|---|---|
| GPU 물리 + 정적 콜라이더 + 표면속도 | 통과 |
| GPU 물리 + 키네마틱 강체 + 표면속도 | 통과 |
| CPU 물리 + 정적 콜라이더 + 표면속도 | 통과 |
| CPU 물리 + 키네마틱 강체 + 표면속도 (**NVIDIA 공식 구성**) | 통과 |

같은 하드웨어(RTX 3090)·같은 워크플로에서 동일한 증상이
[NVIDIA 포럼](https://forums.developer.nvidia.com/t/isaac-lab-collision-fails-on-conveyor-surface-velocity-in-interactivescenecfg-teleoperation-task/359980)
에도 보고되어 있고, NVIDIA 답변은 "GitHub 에 이슈를 올려라" 뿐이었다.

대안으로 시도한 것들도 이 환경에서는 반영되지 않았다.

| 방법 | 결과 |
|---|---|
| `set_external_force_and_torque` (마찰을 힘으로 모델링) | 25 m/s² 를 줘도 미동 없음 |
| `write_root_velocity_to_sim` | 명령 후에도 vy=0 |
| `write_root_pose_to_sim` | **동작함** |

그래서 벨트를 **평범한 정적 콜라이더**로 두고, 벨트에 얹힌 물체의 위치를 매 스텝
`speed × dt` 만큼 전진시킨다(`franka_teleop/conveyor.py`). 게으른 선택이 아니라
이 환경에서 실제로 작동하는 유일한 수단이다. `--conveyor force` / `--conveyor surface`
로 다른 방식을 시도해 볼 수 있게는 남겨 두었다.

블록을 집어 올리면 z 가 "벨트에 얹힌 범위" 를 벗어나 구동 대상에서 빠지므로,
그리퍼와 벨트가 서로 싸우지 않는다.

### 물리는 CPU, Fabric 은 켜 둔다

환경이 1개뿐이라 GPU 물리는 오버헤드만 크다. `--physics-device cpu` 로 바꾸니
제어율이 **4.5Hz → 11Hz** 로 올랐다.

주의할 것은 **Fabric 은 반드시 켜 둬야 한다**는 점이다. CPU 물리라 필요 없을 것 같아
껐더니, `write_root_pose_to_sim` 으로 쓴 위치가 렌더러까지 전달되지 않아
**물리 상태는 움직이는데 화면은 그대로**인 상태가 됐다. 텔레메트리(물리 버퍼)만 보면
정상으로 보여서 "동작한다" 고 착각하기 딱 좋다 — 반드시 프레임 두 장을 비교해
눈으로 확인할 것.

### 컨베이어에 새 물체를 올리려면

두 가지를 지켜야 한다.

1. **태스크의 `contact_object_list` 에 이름을 넣는다.** RoboLab 의 `import_scene` 은
   이 목록에 있는 프림만 씬 엔티티로 만든다. 빠지면 화면에 그려지기만 하고
   컨베이어가 손댈 수 없다.
2. **벨트 폭(150mm)에 맞아야 한다.** YCB 그릇(Ø약 160mm)으로 시험했다가 벨트에
   얹히지 못하고 굴러떨어졌다.

이름은 상관없다 — 벨트에 얹히기만 하면 실린다. 물체 높이도 상관없다. 반높이를
스테이지에서 재서 물체별로 "얹힌 높이" 를 계산하기 때문이다.

### 창고 바닥은 z=0 이 아니다

`warehouse.usd` 는 자기 바닥이 z=0 이지만 이 프로젝트는 테이블 상판이 z=0, 지면이
z=−0.697 이다. `world_assets.py` 에서 창고를 그만큼 내려 바닥을 맞춘다. 씬 USDA 쪽에
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

## 에셋

씬이 쓰는 에셋은 모두 `assets/` 안에 있다. 예전에는 씬 USDA 가
`../../../robolab/assets/...` 로 RoboLab 마운트를 직접 가리켜서 컨테이너의 마운트
배치가 바뀌면 깨졌는데, 지금은 필요한 것만 저장소 안으로 복사해 자립적이다.

```
assets/conveyor/conveyor.usda        컨베이어 (도면 FRS-CV-001 기준, 직접 작성)
assets/scenes/conveyor_pick_place.usda   씬
assets/objects/                      RoboLab YCB 에서 가져온 것 (그릇·바나나 + 텍스처)
```

YCB 전체(152MB)를 넣지 않고 실제로 참조되는 것만 복사했다 — 의존 텍스처는
스테이지를 열어 `Sdf.ValueTypeNames.Asset` 속성을 훑어 확인했다
(bowl → `obj_000013.png`, banana → `obj_000010.png`). 저장소의 에셋 총량은 9.2MB 다.

RoboLab 저장소 자체는 여전히 `~/robolab` 에 있어야 한다 — 로봇(Droid) 설정과 파이썬
패키지를 쓴다. `scripts/container_up.sh` 가 `/workspace` 아래에 나란히 마운트한다.

## 하드웨어 메모

RoboLab 은 48GB+ VRAM 을 권장하지만 이 설정은 RTX 3090(24GB) 한 장을 전제로 한다.
`num_envs=1`, 부가 센서 제거, 뷰포트 카메라 1대만 남긴 상태에서 **실측 9.5GB** 로
여유 있게 들어간다. 제어율은 물리를 CPU 로 옮긴 뒤 약 **11Hz** 다 (GPU 물리일 때는
4.5Hz). 환경이 하나뿐이라 GPU 물리의 고정 오버헤드가 손해였다.
