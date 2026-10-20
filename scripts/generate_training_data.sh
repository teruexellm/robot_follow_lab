#!/bin/bash
# Gazebo から学習データを生成する (spec v2.0 §6 の第 1 段)
#
#   TRAIN_IMAGES=2000 scripts/generate_training_data.sh
#
# 人手の annotation は不要。外部データセットも使わない。
#
# set -u は使わない。ROS の setup.bash が未定義変数を参照するため
# "AMENT_TRACE_SETUP_FILES: unbound variable" で即死する (DEVLOG S-4)。
set -o pipefail

WORKSPACE=${FOLLOWBOT_WORKSPACE:-/workspace}
TRAIN_IMAGES=${TRAIN_IMAGES:-2000}
TRAIN_SEED=${TRAIN_SEED:-42}
OUT_DIR=${OUT_DIR:-$WORKSPACE/data/generated}

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/ros_ws/install/setup.bash"

echo "=== 学習データ生成 ==="
echo "  枚数: $TRAIN_IMAGES / seed: $TRAIN_SEED / 出力: $OUT_DIR"

rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

source "$(dirname "$0")/launch_group.sh"
launch_group_start followbot_bringup training.launch.py /tmp/training_launch.log
trap launch_group_stop EXIT

echo "  Gazebo と Leader の spawn を待機..."
# 画像トピックが流れ始めるのを待つ (固定 sleep に頼らない / spec §53)
for _ in $(seq 60); do
  if timeout -k 2 5 ros2 topic echo --once /training_camera/image_raw \
       > /dev/null 2>&1; then
    echo "  カメラ画像を確認"
    break
  fi
  sleep 2
done

RC=0
python3 "$WORKSPACE/training/generate_dataset.py" \
  --num-images "$TRAIN_IMAGES" --seed "$TRAIN_SEED" --out-dir "$OUT_DIR" || RC=$?

echo "=== 生成結果 ==="
for split in train val; do
  if [ -d "$OUT_DIR/$split/images" ]; then
    echo "  $split: $(ls "$OUT_DIR/$split/images" | wc -l) 枚"
  fi
done
echo "  debug: $(ls "$OUT_DIR/debug" 2>/dev/null | wc -l) 枚"
exit $RC
