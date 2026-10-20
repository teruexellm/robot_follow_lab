#!/bin/bash
# AI を完全自動で学習する (spec v2.0 §6, §33)
#
#   make train-ai
#   docker compose --profile train run --rm trainer
#
# 環境変数:
#   TRAIN_IMAGES   既定 2000   生成枚数 (spec §29)
#   TRAIN_EPOCHS   既定 20     学習 epoch 数
#   TRAIN_SEED     既定 42     乱数シード (spec §32)
#
# CI / smoke 用には小規模設定を使える (spec §64):
#   make test-pipeline
#   TRAIN_IMAGES=100 TRAIN_EPOCHS=2 PIPELINE_TEST=1 make train-ai
#
# PIPELINE_TEST=1 を付けると精度の未達を警告にとどめる。
# spec §64 が「目的は精度ではなく pipeline の成立確認」と定めているため。
# 100 枚 / 2 epoch のモデルが bbox MAE 0.06 を満たすことはない。
# 構造の検査 (checkpoint ができたか / ONNX が読めるか) は落とす。
#
# spec §6 の 4 工程に、demo_world での誤検出検査を加えた 5 工程を順に走る。
# 検査を入れる理由は DEVLOG X-3。spec §35 の validation は training_world
# 由来のデータしか見ないため、デプロイ先 (demo_world) の誤検出を
# 捕まえられない。実際に objectness 0.9 の誤検出を見逃していた。
#
# 人手の annotation も外部データセットも要求しない (spec §6)。
set -u

WORKSPACE=${FOLLOWBOT_WORKSPACE:-/workspace}
export TRAIN_IMAGES=${TRAIN_IMAGES:-2000}
export TRAIN_EPOCHS=${TRAIN_EPOCHS:-20}
export TRAIN_SEED=${TRAIN_SEED:-42}
export PIPELINE_TEST=${PIPELINE_TEST:-}

# `set -u` のもとで ROS の setup.bash を source すると落ちる (DEVLOG S-4)
set +u
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}/opt/mlvenv/lib/python3.12/site-packages"
source "$WORKSPACE/ros_ws/install/setup.bash"
set -u
export FOLLOWBOT_WORKSPACE="$WORKSPACE"
export PYTHONUNBUFFERED=1

echo "############ AI 学習 (spec §6) ############"
echo "  枚数 $TRAIN_IMAGES / epoch $TRAIN_EPOCHS / seed $TRAIN_SEED"
if [ "$PIPELINE_TEST" = "1" ]; then
  echo "  PIPELINE_TEST=1: 精度の未達は警告にとどめる (spec §64)"
fi
echo

run_step() {
  local label="$1"; shift
  echo "---------- $label ----------"
  "$@"
  local rc=$?
  if [ $rc -ne 0 ]; then
    echo
    echo "  FAILED: $label (exit $rc)"
    echo "  ここで止める。後続の工程は前の成果物に依存している"
    exit $rc
  fi
  echo
  return 0
}

# 1. Gazebo training world から画像と annotation を生成 (spec §25-§28)
run_step "1/5 データ生成" \
  bash "$WORKSPACE/scripts/generate_training_data.sh"

# 2. CPU で学習 (spec §24)
run_step "2/5 学習" \
  python3 "$WORKSPACE/training/train_detector.py"

# 3. ONNX へ変換 (spec §34)
run_step "3/5 ONNX 変換" \
  python3 "$WORKSPACE/training/export_onnx.py"

# 4. ONNX の検証 (spec §35)
run_step "4/5 ONNX 検証" \
  python3 "$WORKSPACE/training/validate_model.py"

# 5. demo_world での誤検出検査 (DEVLOG X-3)
#    検証セットが無い環境では skip する。学習自体は成立しているため
if [ -s "$WORKSPACE/data/demo_eval/labels.csv" ]; then
  run_step "5/5 demo_world での誤検出検査" \
    python3 "$WORKSPACE/scripts/check_ai_on_demo.py"
else
  echo "---------- 5/5 demo_world での誤検出検査 ----------"
  echo "  skip (data/demo_eval が無い)"
  echo "  収集するには scripts/capture_demo_eval.sh を実行する。"
  echo "  この検査が無いと、学習データでは precision 1.0 なのに"
  echo "  実機で誤検出するモデルを見逃す (DEVLOG X-3)"
  echo
fi

echo "############ 完了 ############"
ls -lh "$WORKSPACE/models/" | sed 's/^/  /'
echo
if [ "$PIPELINE_TEST" = "1" ]; then
  cat <<'MSG'

  注意: これは pipeline の成立確認用のモデルであって、デモには使えない。
        少量・少 epoch では objectness が動作点 (ai_threshold 0.9999) に
        届かず、検出率 0 になる。make demo を実行しても Follower は
        動かず、validate_output が target_pose 0 件で NG を出す。

        デモ用のモデルは既定値で作る:
            make train-ai            (2000 枚 / 20 epoch / 約 30 分)
        AI 無しで動かすなら:
            make demo-marker
MSG
else
  echo "  デモを実行する: make demo"
fi
