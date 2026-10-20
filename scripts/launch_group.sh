#!/bin/bash
# ros2 launch をプロセスグループごと起動・終了するための共通部品。
#
# なぜ必要か (DEVLOG S-19):
#   ros2 launch を & で起動して kill "$LAUNCH_PID" で止めると、
#   ros2 launch 本体しか死なない。孫である gz sim と parameter_bridge は
#   生き残り、コンテナの PID 1 に里親付けされて走り続ける。
#   train_ai.sh のように 1 つのコンテナで複数の工程を順に回す場合、
#   生成工程の Gazebo が学習工程の裏で CPU を食い続けることになる。
#   4 コアの機械で実測 0.7 コア。症状は「学習が遅い」だけなので気づきにくい。
#
# 使い方:
#   source "$(dirname "$0")/launch_group.sh"
#   launch_group_start followbot_bringup training.launch.py /tmp/launch.log
#   trap launch_group_stop EXIT

LAUNCH_GROUP_PID=""

launch_group_start() {
  local pkg="$1" launch_file="$2" log="${3:-/tmp/launch_group.log}"
  shift 3 2>/dev/null || shift $#
  # setsid で新しいプロセスグループにする。これがないと負の PID で
  # グループ kill できない。
  setsid ros2 launch "$pkg" "$launch_file" "$@" > "$log" 2>&1 &
  LAUNCH_GROUP_PID=$!
}

launch_group_stop() {
  [ -n "$LAUNCH_GROUP_PID" ] || return 0
  # 負の PID はプロセスグループ全体を指す
  kill -TERM -"$LAUNCH_GROUP_PID" 2>/dev/null \
    || kill -TERM "$LAUNCH_GROUP_PID" 2>/dev/null || true
  local i
  for i in $(seq 10); do
    kill -0 -"$LAUNCH_GROUP_PID" 2>/dev/null || break
    sleep 1
  done
  kill -KILL -"$LAUNCH_GROUP_PID" 2>/dev/null || true
  # launch の管理外で残るものを名指しで掃除する
  pkill -KILL -f 'gz sim '                     2>/dev/null || true
  pkill -KILL -f 'ros_gz_bridge/parameter_bridge' 2>/dev/null || true
  LAUNCH_GROUP_PID=""
}
