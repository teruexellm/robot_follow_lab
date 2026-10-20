#!/bin/bash
# ROS 2 環境と followbot ワークスペースを有効化してからコマンドを実行する
set -e
source /opt/ros/jazzy/setup.bash

# ros2 run の console script はシステム python で動くため、
# venv の site-packages を見えるようにする (DEVLOG E-3)
VENV_SITE=/opt/mlvenv/lib/python3.12/site-packages
case ":${PYTHONPATH}:" in
  *":${VENV_SITE}:"*) ;;
  *) export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}${VENV_SITE}" ;;
esac
if [ -f /workspace/ros_ws/install/setup.bash ]; then
  source /workspace/ros_ws/install/setup.bash
fi
exec "$@"
