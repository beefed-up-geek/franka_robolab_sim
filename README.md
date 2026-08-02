# franka_robolab_sim

[NVlabs/RoboLab](https://github.com/NVlabs/RoboLab) 위에 올린 Franka 조작 시뮬레이션.
컨베이어를 타고 흘러오는 물체를 사람이 브라우저에서 집고, 그 시연을 모아 VLA 를
학습시켜 다시 시뮬레이션에서 돌리는 것이 전체 그림이다.

```
env/               시뮬레이션 환경 (Isaac Sim)
data_collection/   시연 데이터 수집 → LeRobot 형식        [미구현]
inference/         학습한 VLA 추론                        [미구현]

docker/  scripts/  세 폴더가 공유하는 컨테이너 인프라
tools/            브라우저 없이 도는 검증 스크립트
```

세 폴더는 **ROS 로만** 이어진다. 환경은 물체·그리퍼 자세를 퍼블리시하고 조작 명령을
구독하며, 수집기와 추론기는 그 인터페이스에만 붙는다. 시뮬레이터를 직접 import 하지
않으므로 같은 코드가 실물 로봇에도 붙는다. (ROS 노드는 아직 없다 — `env/README.md` 참고)

## 빠른 시작

```bash
./scripts/build.sh            # 이미지 빌드 (robolab:teleop 베이스가 없으면 같이 만든다)
./scripts/container_up.sh     # 컨테이너 기동
./scripts/sim_start.sh env_test   # 컨베이어 환경 실행
```

브라우저에서 `http://<서버주소>:8003` 접속. 창을 한 번 클릭해 포커스를 준 뒤 키를 누른다.
정지는 `./scripts/sim_stop.sh`, 로그는 `./scripts/logs.sh`.

동작 검증(브라우저 없이): `python3 tools/test_teleop.py --host <서버주소>`

조작법과 환경 구조는 [`env/README.md`](env/README.md) 에 있다.

## 요구사항

- NVIDIA GPU + `nvidia-container-toolkit`
- `~/robolab` 에 RoboLab 저장소 (git-lfs 로 에셋까지 받은 상태) — 로봇 설정과
  파이썬 패키지를 쓴다. 씬 자산은 이 저장소 안에 자립적으로 들어 있다.
- 디스크 ~50GB, 인터넷 (창고 배경을 NVIDIA 클라우드에서 받아 캐시한다)

## 왜 MoveIt 을 쓰지 않는가

RoboLab 의 `DroidRelIKActionCfg` 가 Isaac Lab 의 `DifferentialInverseKinematicsActionCfg`
(DLS 기반 `DifferentialIKController`)를 그대로 감싸고 있다. 즉 **IK 는 시뮬레이터 안에서
이미 풀린다.** 액션 벡터에 EEF 델타를 넣으면 관절각은 알아서 나온다.

RoboLab 자체에는 텔레오퍼레이션 코드가 없고, Isaac Lab 의 `Se3Keyboard` 는 `carb.input`
(Omniverse 네이티브 입력)을 구독해서 **GUI 창 포커스를 요구**하므로 원격 브라우저에서는
쓸 수 없다. 그래서 필요한 건 IK 구현이 아니라 브라우저 입력을 액션 벡터로 옮기는
얇은 브리지뿐이고, 그게 `env/src/franka_env/` 다.

## 알아둘 것 (여기서 시간을 많이 썼다)

### RobolabEnv 는 리셋하지 않고 "freeze" 한다

`RobolabEnv` 는 정책 벤치마크용이라, 에피소드가 한 번이라도 스텝된 뒤 종료되면
env 를 리셋하는 대신 **freeze** 시킨다.

```python
# robolab/core/environments/env.py
def step(self, action):
    if self._frozen_envs.any():
        action[self._frozen_envs] = 0.0     # 액션이 통째로 0 이 된다
```

그래서 `env.reset()` 만 부르면 **겉보기엔 팔이 홈 자세로 돌아가지만 그 뒤로 키 입력이
전혀 먹지 않는다.** 관측값도 종료 시점에 멈춰 UI 좌표가 얼어붙는다. 해결은
`reset_eval_state()` 로 freeze 를 푸는 것이다 (`franka_env/runner.py:reset_episode`).

```python
end_episode(env)          # 레코더 정리
env.reset_eval_state()    # freeze 해제 (이게 핵심)
obs, _ = env.reset()
```

### 컨베이어는 PhysX 표면 속도로 만들 수 없었다 (공식 방식 포함)

컨베이어의 정석은 `PhysxSurfaceVelocityAPI` 이고 공식
`isaacsim.asset.gen.conveyor` 익스텐션도 같은 API 를 쓴다. NVIDIA 자체 테스트가
요구하는 조건(PhysX 씬 `gpu_dynamics=False`, `broadphase=MBP`, `solver=TGS`,
벨트는 키네마틱 강체)을 전부 맞춘 뒤에도 실패했다.

| 조합 | 결과 |
|---|---|
| GPU 물리 + 정적 콜라이더 | 물체가 벨트를 통과 |
| GPU 물리 + 키네마틱 강체 | 통과 |
| CPU 물리 + 정적 콜라이더 | 통과 |
| CPU 물리 + 키네마틱 강체 (**NVIDIA 공식 구성**) | 통과 |

같은 하드웨어(RTX 3090)·같은 워크플로에서 동일한 증상이
[NVIDIA 포럼](https://forums.developer.nvidia.com/t/isaac-lab-collision-fails-on-conveyor-surface-velocity-in-interactivescenecfg-teleoperation-task/359980)
과 [IsaacLab #4561](https://github.com/isaac-sim/IsaacLab/issues/4561) 에 보고되어
있고 해결책이 없다.

대안도 이 환경에서는 반영되지 않았다.

| 방법 | 결과 |
|---|---|
| `set_external_force_and_torque` | 25 m/s² 를 줘도 미동 없음 |
| `write_root_velocity_to_sim` | 명령 후에도 vy=0 |
| `write_root_pose_to_sim` | **동작함** |

그래서 벨트를 정적 콜라이더로 두고 벨트에 얹힌 물체의 위치를 매 스텝
`speed × dt` 만큼 전진시킨다. 게으른 선택이 아니라 이 환경에서 작동하는 유일한
수단이다. `--conveyor force` / `--conveyor surface` 로 다시 시도해 볼 수 있게는 두었다.

### 물리는 CPU, Fabric 은 켜 둔다

환경이 1개뿐이라 GPU 물리는 오버헤드만 크다. CPU 로 바꾸니 제어율이
**4.5Hz → 11Hz** 로 올랐다.

**Fabric 은 반드시 켜 둬야 한다.** CPU 물리라 필요 없을 것 같아 껐더니
`write_root_pose_to_sim` 으로 쓴 위치가 렌더러에 전달되지 않아 **물리는 움직이는데
화면은 그대로**인 상태가 됐다. 텔레메트리만 보면 정상으로 보여서 착각하기 쉽다 —
반드시 프레임 두 장을 비교해 눈으로 확인할 것.

### 입력 타임아웃은 클라이언트별이어야 한다

"마지막 입력 시각" 을 전역으로 두면 **다른 탭이 보내는 하트비트가 조작 중인
클라이언트의 타임아웃을 갱신해버린다.** 조작하던 브라우저가 죽어도 구경만 하는 탭이
하나 열려 있으면 로봇이 계속 움직인다. 연결마다 `_Client` 를 둔다.

### 컨베이어에 새 물체를 올리려면

1. **태스크의 `contact_object_list` 에 이름을 넣는다.** RoboLab 의 `import_scene` 은
   이 목록에 있는 프림만 씬 엔티티로 만든다. 빠지면 화면에 그려지기만 하고
   컨베이어가 손댈 수 없다.
2. **벨트 폭(150mm)에 맞아야 한다.** YCB 그릇(Ø약 160mm)은 얹히지 못하고 굴러떨어졌다.

이름과 높이는 상관없다 — 반높이를 스테이지에서 재서 물체별로 판정한다.

### 그 밖에

- **태스크의 `contact_object_list` 에 "table" 이 필요하다.** `pick_and_place`
  서브태스크가 `gripper__table` 접촉 센서를 이름으로 찾는다. 빠지면 첫 스텝에서
  `ValueError`.
- **상대 IK 는 명령한 델타를 그대로 따라가지 않는다.** 대략 1/3 수준만 반영되어
  `config.POS_DELTA` 는 실측으로 잡았다.
- **RoboLab `HeadCameraCfg` 는 화면이 90° 돌아간다.** 쿼터니언 성분 순서가 어긋나
  있어 `franka_env/camera.py` 에서 look-at 을 직접 계산해 쓴다.
- **창고 바닥은 z=0 이 아니다.** `warehouse.usd` 는 자기 바닥이 z=0 이지만 이
  프로젝트는 상판이 z=0, 지면이 z=−0.697 이라 `world_assets.py` 에서 내려 맞춘다.

## 하드웨어 메모

RoboLab 은 48GB+ VRAM 을 권장하지만 이 설정은 RTX 3090(24GB) 한 장을 전제로 한다.
`num_envs=1`, 부가 센서 제거, 뷰포트 카메라 1대만 남긴 상태에서 **실측 9.5GB**.
제어율은 약 **11Hz** 다.
