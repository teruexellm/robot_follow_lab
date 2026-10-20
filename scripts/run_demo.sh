#!/bin/bash
# デモを実行して output/<run_id>/ へ成果物を出す (spec v2.0 §49, §60)
#
#   make demo              ai mode (標準)
#   make demo-marker       marker mode
#   make demo-truth        truth mode (デバッグ専用)
#
#   docker compose up demo
#
# 環境変数:
#   SCENARIO         既定 occlusion_test  (spec §49)
#   PERCEPTION_MODE  既定 ai              (spec §19)
#   SIM_DURATION     既定 300             simulation 秒 (spec §49)
#   RUN_ID           既定 日時            output/<run_id>/ の名前
#   MAKE_VIDEO       既定 true            combined.mp4 を作るか (spec §57)
#
# 中身は demo.launch.py にある (spec §52)。このスクリプトの仕事は
#   * ai mode でモデルが無い場合に spec §36 の書式で落とす
#   * run_id を決める
#   * 実行後に validate_output.sh で成果物を検査する
# の 3 つ。
set -u

WORKSPACE=${FOLLOWBOT_WORKSPACE:-/workspace}
SCENARIO=${SCENARIO:-occlusion_test}
PERCEPTION_MODE=${PERCEPTION_MODE:-ai}
SIM_DURATION=${SIM_DURATION:-300}
MAKE_VIDEO=${MAKE_VIDEO:-true}
RUN_ID=${RUN_ID:-$(date +%Y%m%d_%H%M%S)}
MODEL=${ONNX_MODEL:-$WORKSPACE/models/leader_detector.onnx}

# ---- ai mode でモデルが無い場合 (spec §36) ----
# make demo が暗黙に AI 学習を始めてはならない
if [ "$PERCEPTION_MODE" = "ai" ] && [ ! -f "$MODEL" ]; then
  cat <<MSG

ERROR: $MODEL not found.

Run:

    make train-ai

or:

    PERCEPTION_MODE=marker make demo
MSG
  exit 2
fi

# `set -u` のもとで ROS の setup.bash を source すると未定義変数で落ちる
# (DEVLOG S-4)。source の前後だけ外す
set +u
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}/opt/mlvenv/lib/python3.12/site-packages"
source "$WORKSPACE/ros_ws/install/setup.bash"
set -u
export FOLLOWBOT_WORKSPACE="$WORKSPACE"
export PYTHONUNBUFFERED=1

OUT="$WORKSPACE/output/$RUN_ID"
echo "############ デモ ############"
echo "  scenario   : $SCENARIO"
echo "  perception : $PERCEPTION_MODE"
echo "  duration   : $SIM_DURATION simulation 秒"
echo "  run_id     : $RUN_ID"
echo "  出力先     : $OUT"
if [ "$PERCEPTION_MODE" = "truth" ]; then
  echo "  注意: truth mode は Ground Truth を使うデバッグ専用 (spec §20)。"
  echo "        最終デモの成果ではない"
fi
echo

# demo.launch.py が duration 到達で自分から終了する。
# 固定 sleep で止めると RTF が 1.0 でない環境で意味が変わる (DEVLOG M-1)
ros2 launch followbot_bringup demo.launch.py \
  scenario:="$SCENARIO" \
  perception_mode:="$PERCEPTION_MODE" \
  duration:="$SIM_DURATION" \
  run_id:="$RUN_ID" \
  make_video:="$MAKE_VIDEO"
LAUNCH_RC=$?

echo
echo "############ 成果物の検査 (spec §60) ############"
bash "$WORKSPACE/scripts/validate_output.sh" "$RUN_ID"
VALIDATE_RC=$?

echo
if [ $LAUNCH_RC -ne 0 ]; then
  echo "  NOTE: launch の終了コードは $LAUNCH_RC。"
  echo "        shutdown 時のシグナルで 0 以外になることがある。"
  echo "        判断は上の成果物の検査結果で行う"
fi
if [ $VALIDATE_RC -eq 0 ]; then
  echo "  RESULT: PASS  -> $OUT"
  if [ -s "$OUT/summary.json" ]; then
    python3 - "$OUT/summary.json" <<'PY'
import json, sys
s = json.load(open(sys.argv[1]))
print(f"    follow RMSE {s.get('follow_rmse_m')} m / "
      f"検出率 {s.get('target_detected_fraction')} / "
      f"衝突 {s.get('collision_count')}")
PY
  fi
else
  echo "  RESULT: FAIL  成果物が揃っていない"
fi
exit $VALIDATE_RC
