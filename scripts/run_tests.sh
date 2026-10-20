#!/bin/bash
# unit test をまとめて実行する (spec v2.0 §63)
#
#   make test
#   docker compose --profile test run --rm test
#
# ROS も Gazebo も起動せずに走るテストだけを対象にする。
# 統合検証 (Gazebo を起動するもの) は smoke-test / demo 側で行う。
#
# set -u は使わない (ROS の setup.bash が未定義変数を参照する / DEVLOG S-4)
set -o pipefail

WORKSPACE=${FOLLOWBOT_WORKSPACE:-/workspace}

source /opt/ros/jazzy/setup.bash
if [ -f "$WORKSPACE/ros_ws/install/setup.bash" ]; then
  source "$WORKSPACE/ros_ws/install/setup.bash"
else
  echo "ERROR: ワークスペースがビルドされていない。make build を実行する"
  exit 2
fi
# ros2 run の console script はシステム python で動くため (DEVLOG E-3)
export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}/opt/mlvenv/lib/python3.12/site-packages"
export FOLLOWBOT_SCENARIO_DIR="$WORKSPACE/scenarios"

RC=0

echo "############ unit test (spec §63) ############"
# test ディレクトリは**列挙せず glob で拾う**。
# 以前は 4 つを直書きしていたため、experiment_logger を足したときに
# その 23 件が黙って実行されないままだった。
# glob が空振りすると全部素通りするので、最低数を確認する。
TEST_DIRS=()
for d in "$WORKSPACE"/ros_ws/src/*/test; do
  [ -d "$d" ] && TEST_DIRS+=("$d")
done
TEST_DIRS+=("$WORKSPACE/training/tests")

if [ ${#TEST_DIRS[@]} -lt 5 ]; then
  echo "  NG  test ディレクトリが ${#TEST_DIRS[@]} 個しか見つからない"
  echo "      (ros_ws/src/*/test を glob した結果が少なすぎる)"
  RC=1
fi
echo "  対象: ${#TEST_DIRS[@]} ディレクトリ"
python3 -m pytest "${TEST_DIRS[@]}" -q || RC=$?

echo
echo "############ scenario の座標検査 ############"
# waypoint が障害物の内接円内に無いことを機械的に確認する (DEVLOG N-6)
python3 "$WORKSPACE/scripts/check_scenarios.py" \
  --world "$WORKSPACE/worlds/demo_world.sdf" \
  --scenario-dir "$WORKSPACE/scenarios" | tail -5 || RC=$?

echo
echo "############ map が world と一致しているか ############"
# make_map.py を再実行して、コミット済みの map と差分が無いことを確認する
TMP_MAP=$(mktemp -d)
python3 "$WORKSPACE/scripts/make_map.py" --out-dir "$TMP_MAP" > /dev/null
if cmp -s "$TMP_MAP/demo_map.pgm" \
          "$WORKSPACE/ros_ws/src/followbot_bringup/maps/demo_map.pgm"; then
  echo "  OK  demo_map.pgm は demo_world.sdf と一致している"
else
  echo "  NG  demo_map.pgm が world と一致しない。make_map.py を再実行する"
  RC=1
fi
rm -rf "$TMP_MAP"

echo
echo
echo "############ DEVLOG の整合 ############"
# 目次のずれと、本文が無い参照を検出する。どちらも実際に
# 2 回やった抜けで、機械的に検出できる (T-2, S-10)
python3 "$WORKSPACE/scripts/check_devlog.py" || RC=$?

echo
echo
echo "############ compose の環境変数 ############"
# スクリプトが読む変数が compose から届くか。書き忘れると
# 無言で既定値が使われ、利用者は指定が効かないことに気付かない
python3 "$WORKSPACE/scripts/check_compose_env.py" || RC=$?

echo "############ launch の後始末 ############"
# 背景起動した ros2 launch をプロセスグループごと落としているか。
# 親だけ kill すると gz sim が生き残り、次の工程の CPU を食う。
# 症状が「なんとなく遅い」としてしか出ないので機械的に塞ぐ
python3 "$WORKSPACE/scripts/check_launch_cleanup.py" || RC=$?
# 真偽値を "true" 完全一致で読むと MAKE_VIDEO=1 が無言で無効化される
python3 "$WORKSPACE/scripts/check_launch_bool.py" || RC=$?
# image に依存しない make ターゲットは古いコードを走らせる
python3 "$WORKSPACE/scripts/check_make_image.py" || RC=$?
# spec §N の参照先が仕様書に実在するか。番号の書き間違いは無言で残る
python3 "$WORKSPACE/scripts/check_spec_refs.py" || RC=$?
# スキルは索引なので、参照が切れると価値がゼロになる
python3 "$WORKSPACE/scripts/check_skills.py" || RC=$?

echo "############ ドキュメントの整合 ############"
# spec §65/§66/§67 の見出しと、make ターゲット・ファイルパスの
# 参照が解決するかを検査する。存在しないコマンドを書いたドキュメントは
# 読んだ人が最初の 5 分で詰まる
python3 "$WORKSPACE/scripts/check_docs.py" || RC=$?

echo "############ demo_world での AI 誤検出検査 (DEVLOG X-3) ############"
# 学習データ由来の validation では捕まらない誤検出を検出する。
# モデルか検証セットが無い環境では skip する (unit test は常に通るべき)。
if [ -f "$WORKSPACE/models/leader_detector.onnx" ] \
   && [ -f "$WORKSPACE/data/demo_eval/labels.csv" ]; then
  python3 "$WORKSPACE/scripts/check_ai_on_demo.py" \
    --data-dir "$WORKSPACE/data/demo_eval" \
    --model "$WORKSPACE/models/leader_detector.onnx" | tail -12 || RC=$?
else
  echo "  skip (models/leader_detector.onnx か data/demo_eval が無い)"
fi

echo
echo "############ 結果 ############"
if [ $RC -eq 0 ]; then
  echo "  RESULT: PASS"
else
  echo "  RESULT: FAIL (exit $RC)"
fi
exit $RC
