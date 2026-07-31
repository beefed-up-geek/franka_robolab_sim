#!/usr/bin/env bash
# 텔레오퍼레이션 로그 따라가기
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
tail -f "$REPO_DIR/logs/teleop.log"
