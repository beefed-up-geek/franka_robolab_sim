# scripts/host — gty 호스트에서 도는 운영 스크립트

컨테이너 밖(호스트)에서 수집·변환·평가·업로드를 굴리는 도구들이다.
시뮬레이션·수집기·추론기 코드는 env/, data_collection/, inference/ 에 있고,
여기는 그것들을 **엮어 돌리는** 쪽이다.

| 파일 | 역할 |
|---|---|
| post_task3_v9.sh | task3 수집 → lerobot 변환 → a4 전송 파이프라인 |
| eval_task3_v9.sh | 학습 완료 감시 → 체크포인트 회수 → 롤아웃 평가 |
| demo_all.sh | 학습된 모델 전부 순차 시연 (환경 자동 전환) |
| run_abs.sh | abs 모델 단건 롤아웃 |
| trace_infer.sh | 추론 중 EEF 궤적 덤프 (診断) |
| openloop_v3.py | 개루프 액션 재생 진단 |
| ingest.py | 수집 원본 → lerobot v3 (abs/delta) |
| ingest_lang.py | 수집 원본 → VLA_lang steering command 합성 데이터셋 |
| upload_safe2026.py | 최종 모델 6종 → HF safe-2026 업로드 |
| cleanup.sh | 고아 수집기·추론기 프로세스 정리 |

경로 규약: 스크립트들은 ~/franka_robolab_sim, ~/_model, ~/_lerobot 절대 경로를
참조한다 — 홈에 코드가 아니라 **산출물 디렉터리**(_model/_lerobot)만 남기는 구조.
