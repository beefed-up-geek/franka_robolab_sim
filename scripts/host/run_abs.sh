#!/usr/bin/env bash
# task2 를 절대 EEF 목표(abs)로 변환해 a4 로 보낸다 — 상대값 누적 오차 회피용.
set -uo pipefail
~/hfenv/bin/python ~/ingest.py --src ~/franka_robolab_sim/_data/task2/main \
    --dst ~/_lerobot/task2_abs --action abs > /tmp/ingest_task2abs.log 2>&1
rsync -rlD --no-t --delete ~/_lerobot/task2_abs/ \
    a4:/mnt/nas/gty/ICRA/train/data/task2_abs/ > /tmp/rsync_task2abs.log 2>&1
echo ABS_DONE >> /tmp/ingest_task2abs.log
