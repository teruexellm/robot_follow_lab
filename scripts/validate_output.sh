#!/bin/bash
# output/<run_id>/ の成果物が spec §60 を満たしているか検査する。
#
#   make validate-output
#   scripts/validate_output.sh [run_id]
#
# run_id を省略すると output/ の最新を見る。
#
# 動画を作らずに実行した run (MAKE_VIDEO=false) では combined.mp4 を
# 要求しない (DEVLOG S-28)。README が `MAKE_VIDEO=false make demo` を
# 案内しているのに、そのとおり実行した run が FAIL になっていた。
# 判定は frames/ の有無で行う。MAKE_VIDEO=false なら素材も残らないため:
#   frames/ も mp4 も無い  -> 動画を作らない指定だった。要求しない
#   frames/ はあるが mp4 が無い -> 合成に失敗した。NG
set -u

WORKSPACE=${FOLLOWBOT_WORKSPACE:-/workspace}
OUTPUT_ROOT="$WORKSPACE/output"

if [ $# -ge 1 ]; then
  RUN_DIR="$OUTPUT_ROOT/$1"
else
  RUN_DIR=$(ls -dt "$OUTPUT_ROOT"/*/ 2>/dev/null | head -1)
  RUN_DIR=${RUN_DIR%/}
fi

if [ -z "${RUN_DIR:-}" ] || [ ! -d "$RUN_DIR" ]; then
  echo "ERROR: 検査する実験結果が無い ($OUTPUT_ROOT)"
  echo "  先に make demo を実行する"
  exit 2
fi

echo "############ output の検査: $RUN_DIR ############"
RC=0

check_file() {
  local path="$1" label="$2" optional="${3:-no}"
  if [ -s "$RUN_DIR/$path" ] || { [ -d "$RUN_DIR/$path" ] && [ -n "$(ls -A "$RUN_DIR/$path" 2>/dev/null)" ]; }; then
    echo "  OK   $label"
  elif [ "$optional" = "optional" ]; then
    echo "  --   $label (未生成。Phase 16 の成果物)"
  else
    echo "  NG   $label が無いか空"
    RC=1
  fi
}

echo "--- spec §60 の構造 ---"
check_file telemetry.csv  "telemetry.csv"
check_file events.csv     "events.csv"
check_file summary.json   "summary.json"
check_file scenario.yaml  "scenario.yaml"
check_file environment.txt "environment.txt"
check_file rosbag         "rosbag/"

# 動画を作らない指定で実行された run かどうか
VIDEO_EXPECTED=1
if [ ! -d "$RUN_DIR/frames" ] && [ ! -s "$RUN_DIR/combined.mp4" ]; then
  VIDEO_EXPECTED=0
fi
if [ "$VIDEO_EXPECTED" = 1 ]; then
  check_file combined.mp4 "combined.mp4"
else
  echo "  --   combined.mp4 (動画を作らない指定で実行された run)"
fi

echo
echo "--- telemetry.csv の列 (spec §55) ---"
python3 - "$RUN_DIR" <<'PY' || RC=1
import csv, sys
from pathlib import Path

run = Path(sys.argv[1])
required = ["sim_time", "true_distance_m", "estimated_distance_m", "detected",
            "confidence", "follow_error_m", "follower_vx", "follower_wz",
            "nav2_status", "target_state"]
rows = list(csv.DictReader(open(run / "telemetry.csv")))
if not rows:
    print("  NG   telemetry.csv に行が無い")
    sys.exit(1)
header = list(rows[0].keys())
missing = [c for c in required if c not in header]
if missing:
    print(f"  NG   spec §55 の必須列が無い: {missing}")
    sys.exit(1)
print(f"  OK   必須 {len(required)} 列すべてある ({len(rows)} 行)")

# 列があっても中身が空では意味がない。主要な列の充足率を見る
rc = 0
for col in required[1:]:
    filled = sum(1 for r in rows if r[col] not in ("", None))
    ratio = filled / len(rows)
    if ratio < 0.5:
        print(f"  NG   {col} が {ratio:.0%} しか埋まっていない")
        rc = 1
print(f"  OK   主要な列が埋まっている" if rc == 0 else "", end="\n" if rc == 0 else "")
sys.exit(rc)
PY

echo
echo "--- summary.json のフィールド (spec §56) ---"
python3 - "$RUN_DIR" <<'PY' || RC=1
import json, sys
from pathlib import Path

required = ["scenario", "perception", "duration_sec", "desired_follow_distance_m",
            "mean_follow_distance_m", "follow_rmse_m", "target_detected_fraction",
            "target_lost_count", "collision_count"]
s = json.load(open(Path(sys.argv[1]) / "summary.json"))
missing = [k for k in required if k not in s]
if missing:
    print(f"  NG   必須フィールドが無い: {missing}")
    sys.exit(1)
print(f"  OK   spec §56 の {len(required)} フィールドすべてある")
print(f"       scenario={s['scenario']} perception={s['perception']} "
      f"duration={s['duration_sec']}s")
print(f"       RMSE={s['follow_rmse_m']} m / 検出率={s['target_detected_fraction']} "
      f"/ 衝突={s['collision_count']}")
if s.get("samples", 1) == 0:
    print("  NG   サンプルが 1 つも無い")
    sys.exit(1)

# 誰も動いていない run は、数字だけ見ると最良に見える (DEVLOG S-30)。
# Leader が静止していれば Follower は目標距離を保ったままなので
# RMSE は小さく、検出率は 1.0、衝突は 0 になる。
# 走行距離を見ないと区別できない。
lead = s.get("leader_path_m")
foll = s.get("follower_path_m")
if lead is not None and foll is not None and s["duration_sec"] > 20:
    if lead < 0.5 and foll < 0.5:
        print(f"  NG   どちらのロボットも動いていない "
              f"(leader {lead} m / follower {foll} m)")
        print("       起動に失敗している。この run の RMSE と検出率は読めない")
        print("       ログで 'Failed to send goal response' を探す (DEVLOG N-13)")
        sys.exit(1)
    if lead < 0.5:
        print(f"  NG   Leader が動いていない (leader_path_m={lead})")
        sys.exit(1)
    print(f"       走行距離 leader {lead} m / follower {foll} m")
sys.exit(0)
PY

echo
echo "--- combined.mp4 の仕様 (spec §57) ---"
if [ -s "$RUN_DIR/combined.mp4" ] && command -v ffprobe > /dev/null 2>&1; then
  PROBE=$(ffprobe -v error -select_streams v:0 \
    -show_entries stream=width,height,codec_name,avg_frame_rate,nb_frames \
    -of default=noprint_wrappers=1 "$RUN_DIR/combined.mp4" 2>&1)
  W=$(echo "$PROBE" | sed -n 's/^width=//p')
  H=$(echo "$PROBE" | sed -n 's/^height=//p')
  CODEC=$(echo "$PROBE" | sed -n 's/^codec_name=//p')
  RATE=$(echo "$PROBE" | sed -n 's/^avg_frame_rate=//p')
  [ "$W/$H" = "1280/720" ] && echo "  OK   解像度 ${W}x${H}" \
    || { echo "  NG   解像度が ${W}x${H} (1280x720 であるべき)"; RC=1; }
  [ "$CODEC" = "h264" ] && echo "  OK   codec $CODEC" \
    || { echo "  NG   codec が $CODEC (h264 であるべき)"; RC=1; }
  [ "$RATE" = "10/1" ] && echo "  OK   fps $RATE" \
    || { echo "  NG   fps が $RATE (10/1 であるべき)"; RC=1; }
  echo "  $(echo "$PROBE" | sed -n 's/^nb_frames=/フレーム数 /p')"
elif [ "$VIDEO_EXPECTED" = 0 ]; then
  echo "  --   検査しない (MAKE_VIDEO=false で実行された run)"
elif [ ! -s "$RUN_DIR/combined.mp4" ]; then
  echo "  NG   combined.mp4 が無い"
  RC=1
else
  echo "  skip (ffprobe が無い)"
fi

echo
echo "--- rosbag の内容 (spec §54) ---"
if command -v ros2 > /dev/null 2>&1; then
  BAG_INFO=$(ros2 bag info "$RUN_DIR/rosbag" 2>&1)
  for topic in /follower/odom /follower/scan /follower/target_detection \
               /follower/target_pose /follower/cmd_vel /leader/odom; do
    if echo "$BAG_INFO" | grep -q "Topic: $topic "; then
      COUNT=$(echo "$BAG_INFO" | grep "Topic: $topic " | sed 's/.*Count: \([0-9]*\).*/\1/')
      if [ "${COUNT:-0}" -gt 0 ]; then
        echo "  OK   $topic ($COUNT 件)"
      else
        echo "  NG   $topic が 0 件"
        RC=1
      fi
    else
      echo "  NG   $topic が bag に無い"
      RC=1
    fi
  done
else
  echo "  skip (ros2 が無い)"
fi

echo
echo "############ 結果 ############"
if [ $RC -eq 0 ]; then
  echo "  RESULT: PASS"
else
  echo "  RESULT: FAIL"
fi
exit $RC
