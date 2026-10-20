#!/bin/bash
# demo_world の Follower 視界を真値付きで収集する (DEVLOG X-3 のゲート用)。
#
#   docker compose run --rm test /workspace/scripts/capture_demo_eval.sh
#
# marker mode で走らせる。AI の誤検出に引きずられずに Follower が実際に
# 追従し、視界が多様になる。収集物は data/demo_eval/ に入り、学習には使わない。
set -eu

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}/opt/mlvenv/lib/python3.12/site-packages"
source /workspace/ros_ws/install/setup.bash
export FOLLOWBOT_WORKSPACE=/workspace
export PYTHONUNBUFFERED=1

SCENARIO=${SCENARIO:-occlusion_test}
SETTLE=${SETTLE:-125}
DURATION=${DURATION:-260}
PERIOD=${PERIOD:-1.0}
OUT=${OUT:-/workspace/data/demo_eval}

echo "scenario=$SCENARIO / 整定 ${SETTLE}s / 収集 ${DURATION}s / 間隔 ${PERIOD}s -> $OUT"
source "$(dirname "$0")/launch_group.sh"
launch_group_start followbot_bringup follow_stack.launch.py \
  /tmp/capture_demo_eval.log \
  scenario:="$SCENARIO" perception_mode:=marker
trap launch_group_stop EXIT

sleep "$SETTLE"
rm -rf "$OUT"
timeout -k 10 $((DURATION + 40)) python3 -u /workspace/scripts/capture_demo_eval.py \
  "$OUT" "$DURATION" "$PERIOD"
