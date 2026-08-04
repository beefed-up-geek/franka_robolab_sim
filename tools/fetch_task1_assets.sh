#!/usr/bin/env bash
# task1 공구 에셋 준비 — robolab 체크아웃에서 복사한다 (약 300MB 라 git 에 안 넣음).
#
# 공구마다 자기 textures/ 를 통째로 갖게 한다. 처음에 두 세트의 textures 를 한
# 폴더에 합쳤더니 같은 이름끼리 충돌해 드릴이 흰 모델로 나왔다.
set -euo pipefail
RL="${1:-$HOME/robolab}/assets/objects"
DST="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/env/asset/objects/tools"
mkdir -p "$DST"/{hammer_7,cordless_drill,scissors}
cp "$RL/handal/hammer_7.usd"       "$DST/hammer_7/"
cp -r "$RL/handal/textures"        "$DST/hammer_7/textures"
cp "$RL/ycb/cordless_drill.usd"    "$DST/cordless_drill/"
cp -r "$RL/ycb/textures"           "$DST/cordless_drill/textures"
cp "$RL/ycb/scissors.usd"          "$DST/scissors/"
cp -r "$RL/ycb/textures"           "$DST/scissors/textures"
echo "완료: $DST"
