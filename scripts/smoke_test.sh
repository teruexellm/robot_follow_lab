#!/bin/bash
# 短時間の動作確認 (spec v2.0 §62)
#
#   make smoke-test
#   docker compose --profile test run --rm smoke
#
# 環境変数:
#   SCENARIO         既定 simple_loop
#   PERCEPTION_MODE  既定 marker        ONNX の有無に依存しないため
#   SIM_DURATION     既定 45            simulation 秒 (spec §62 は 30-60)
#
# spec §62 が挙げる確認項目を 1 つずつ検査する。
#
#     Gazebo 起動 / Leader spawn / Follower spawn
#     ros2_control active
#     Camera publish / LiDAR publish / Odometry publish
#     Nav2 active
#     Leader movement / Follower movement
#     output 生成
#
# marker mode を既定にするのは、学習済みモデルが無い状態でも
# 回せるようにするため。spec §62 の目的は配管の確認であって
# AI の精度確認ではない。
set -u

WORKSPACE=${FOLLOWBOT_WORKSPACE:-/workspace}
SCENARIO=${SCENARIO:-simple_loop}
PERCEPTION_MODE=${PERCEPTION_MODE:-marker}
SIM_DURATION=${SIM_DURATION:-45}
RUN_ID=${RUN_ID:-smoke_$(date +%Y%m%d_%H%M%S)}

set +u
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}/opt/mlvenv/lib/python3.12/site-packages"
source "$WORKSPACE/ros_ws/install/setup.bash"
set -u
export FOLLOWBOT_WORKSPACE="$WORKSPACE"
export PYTHONUNBUFFERED=1

OUT="$WORKSPACE/output/$RUN_ID"
LOG=/tmp/smoke_launch.log
RC=0

echo "############ smoke test (spec §62) ############"
echo "  scenario $SCENARIO / perception $PERCEPTION_MODE"
echo "  duration $SIM_DURATION simulation 秒 / run_id $RUN_ID"
echo

# 動画は作らない。配管の確認に 1280x720 の合成は要らない
ros2 launch followbot_bringup demo.launch.py \
  scenario:="$SCENARIO" \
  perception_mode:="$PERCEPTION_MODE" \
  duration:="$SIM_DURATION" \
  run_id:="$RUN_ID" \
  make_video:=false > "$LOG" 2>&1
echo "  launch 終了 (exit $?)"
echo

check() {
  local label="$1" ok="$2" detail="${3:-}"
  if [ "$ok" = "yes" ]; then
    printf "  OK   %-34s %s\n" "$label" "$detail"
  else
    printf "  NG   %-34s %s\n" "$label" "$detail"
    RC=1
  fi
}

# grep -c は 0 件のとき "0" を出して exit 1 する。
# `|| echo 0` を付けると 0 が二重に出て [ の比較が壊れる。
# `|| true` なら grep の出した 0 だけが残る
count() { grep -c "$1" "$LOG" 2>/dev/null || true; }

echo "---------- 起動 ----------"
# /clock が流れていれば Gazebo は動いていた
CLOCK=$([ "$(count 'use_sim_time')" -gt 0 ] && echo yes || echo no)
check "Gazebo 起動" \
  "$([ "$(count 'Activating component')" -gt 0 ] && echo yes || echo no)" \
  "controller_manager がコンポーネントを activate"
check "Leader spawn" \
  "$([ "$(count 'leader.controller_manager')" -gt 0 ] && echo yes || echo no)"
check "Follower spawn" \
  "$([ "$(count 'follower.controller_manager')" -gt 0 ] && echo yes || echo no)"

echo "---------- ros2_control (spec §9) ----------"
for ns in leader follower; do
  for ctrl in joint_state_broadcaster diff_drive_controller; do
    check "$ns/$ctrl active" \
      "$([ "$(count "Activating controllers: \[ $ctrl ]")" -gt 0 ] && echo yes || echo no)"
  done
done

echo "---------- Nav2 (spec §42) ----------"
check "localization active (2 機)" \
  "$([ "$(count 'Activating amcl')" -ge 2 ] && echo yes || echo no)" \
  "$(count 'Activating amcl') 回"
check "navigation active (2 機)" \
  "$([ "$(count 'Activating bt_navigator')" -ge 2 ] && echo yes || echo no)" \
  "$(count 'Activating bt_navigator') 回"
check "navigate_to_pose を待ち続けていない" \
  "$([ "$(count 'navigate_to_pose を待機中')" -eq 0 ] && echo yes || echo no)" \
  "警告 $(count 'navigate_to_pose を待機中') 回 (DEVLOG N-16)"
check "lifecycle の応答取りこぼし無し" \
  "$([ "$(count 'failed to send response')" -eq 0 ] && echo yes || echo no)" \
  "$(count 'failed to send response') 回"

echo "---------- 成果物 (spec §60) ----------"
if [ -s "$OUT/telemetry.csv" ]; then
  python3 - "$OUT" <<'PY'
import csv, json, sys
from pathlib import Path

run = Path(sys.argv[1])
rows = list(csv.DictReader(open(run / "telemetry.csv")))
summary = json.load(open(run / "summary.json"))


def filled(col):
    return sum(1 for r in rows if r.get(col) not in ("", None))


def show(label, ok, detail=""):
    print(f"  {'OK  ' if ok else 'NG  '} {label:<34} {detail}")
    return 0 if ok else 1


rc = 0
# spec §62: Camera / LiDAR / Odometry が publish されていること。
# telemetry の列が埋まっていれば購読できている
rc |= show("Camera publish (検出が動いた)",
           filled("confidence") > len(rows) * 0.5,
           f"{filled('confidence')}/{len(rows)} 行")
rc |= show("Odometry publish",
           filled("follower_vx") > len(rows) * 0.5,
           f"{filled('follower_vx')}/{len(rows)} 行")
rc |= show("LiDAR publish (target_pose が出た)",
           filled("estimated_distance_m") > 0,
           f"{filled('estimated_distance_m')}/{len(rows)} 行")
# spec §62: Leader movement / Follower movement
rc |= show("Leader movement", summary.get("leader_path_m", 0) > 1.0,
           f"{summary.get('leader_path_m')} m")
rc |= show("Follower movement", summary.get("follower_path_m", 0) > 0.5,
           f"{summary.get('follower_path_m')} m")
rc |= show("衝突していない", summary.get("collision_count", 1) == 0,
           f"{summary.get('collision_count')} 回")
sys.exit(1 if rc else 0)
PY
  [ $? -ne 0 ] && RC=1
else
  check "telemetry.csv 生成" no "$OUT に無い"
fi

# combined.mp4 は作っていないので、その行だけ除いて判定する。
# 2 回走らせないよう結果を変数に取る
VALIDATE_OUT=$(bash "$WORKSPACE/scripts/validate_output.sh" "$RUN_ID" 2>&1)
echo "$VALIDATE_OUT" | grep -E "^  (OK|NG|--)" | grep -v combined | sed 's/^/  /'
if echo "$VALIDATE_OUT" | grep -v combined | grep -q "  NG "; then
  RC=1
fi

echo
echo "############ 結果 ############"
if [ $RC -eq 0 ]; then
  echo "  RESULT: PASS"
else
  echo "  RESULT: FAIL"
  echo "  ログ: $LOG"
fi
exit $RC
