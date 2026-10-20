"""telemetry / summary の算出 (spec v2.0 §55, §56)。

ROS に依存しない純粋関数として切り出してある。サンプル列を渡すだけで
検証できるので、Gazebo を起動せずに境界条件まで試験できる
(spec §63 の方針。状態機械を純粋関数にしたのと同じ理由)。

**Ground Truth 距離は評価専用である** (spec §55)。この module が扱う
`true_distance_m` は telemetry と summary の算出にのみ使い、
Follower の制御には一切入力しない。制御側 (follower_tracking) は
`/ground_truth/...` を購読していない (spec §18, §70)。
"""

from __future__ import annotations

import math

# telemetry.csv の列。spec §55 の必須 10 項目を先頭に置き、
# 解析と動画 (Phase 16) で要る値を後ろに足す。
# 先頭を固定しておくと、列が増えても読む側の互換が保てる。
COLUMNS = [
    # --- spec §55 の必須項目 ---
    "sim_time",                 # simulation time [s]
    "true_distance_m",          # true leader/follower distance (評価専用)
    "estimated_distance_m",     # estimated target distance
    "detected",                 # AI detected (0/1)
    "confidence",               # AI confidence
    "follow_error_m",           # follow error = true_distance - desired
    "follower_vx",              # Follower velocity [m/s]
    "follower_wz",              # Follower angular velocity [rad/s]
    "nav2_status",              # Nav2 state
    "target_state",             # target state (INIT/DETECTED/LOST/SEARCHING)
    # --- 解析・動画のための追加項目 ---
    "leader_x", "leader_y",
    "follower_x", "follower_y",
    "target_x", "target_y",
    "estimation_error_m",       # target_pose と Leader 真値の距離
    "lost_count", "redetect_count",
    # 動画 (spec §59) で Follower カメラに bbox を描くために要る。
    # 正規化座標 (0-1)。detected=0 の行は空
    "bbox_center_x", "bbox_center_y", "bbox_width", "bbox_height",
]

EVENT_COLUMNS = ["sim_time", "event", "detail"]


def follow_error(true_distance: float | None, desired: float) -> float | None:
    """追従誤差。正なら離れすぎ、負なら近づきすぎ。"""
    if true_distance is None:
        return None
    return true_distance - desired


def distance(a: tuple[float, float] | None,
             b: tuple[float, float] | None) -> float | None:
    if a is None or b is None:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _values(samples: list[dict], key: str) -> list[float]:
    return [s[key] for s in samples if s.get(key) is not None]


def count_collisions(samples: list[dict], threshold: float) -> int:
    """衝突の回数。

    真値距離が threshold を下回った**区間の数**を数える。サンプルごとに
    数えると 10 Hz で 1 回の接触が何十回にもなるため、立ち下がりの
    エッジだけを数える。
    """
    count = 0
    inside = False
    for s in samples:
        d = s.get("true_distance_m")
        if d is None:
            continue
        if d < threshold:
            if not inside:
                count += 1
                inside = True
        else:
            inside = False
    return count


def summarize(samples: list[dict], *, scenario: str, perception: str,
              desired: float, collision_threshold: float) -> dict:
    """summary.json の中身を作る (spec §56)。

    spec §56 の例が挙げる 9 項目を必ず含め、診断に要る値を後ろに足す。
    """
    if not samples:
        return {
            "scenario": scenario,
            "perception": perception,
            "duration_sec": 0.0,
            "desired_follow_distance_m": desired,
            "samples": 0,
            "note": "サンプルが 1 つも取れなかった",
        }

    times = [s["sim_time"] for s in samples]
    true_d = _values(samples, "true_distance_m")
    est_err = _values(samples, "estimation_error_m")
    detected = [s["detected"] for s in samples if s.get("detected") is not None]

    states = [s.get("target_state") for s in samples]
    histogram: dict[str, int] = {}
    for st in states:
        if st:
            histogram[st] = histogram.get(st, 0) + 1

    lost = [s["lost_count"] for s in samples if s.get("lost_count") is not None]
    redetect = [s["redetect_count"] for s in samples
                if s.get("redetect_count") is not None]

    mean = sum(true_d) / len(true_d) if true_d else None
    rmse = (math.sqrt(sum((d - desired) ** 2 for d in true_d) / len(true_d))
            if true_d else None)

    summary = {
        # --- spec §56 の例が挙げる項目 ---
        "scenario": scenario,
        "perception": perception,
        "duration_sec": round(times[-1] - times[0], 3),
        "desired_follow_distance_m": desired,
        "mean_follow_distance_m": round(mean, 4) if mean is not None else None,
        "follow_rmse_m": round(rmse, 4) if rmse is not None else None,
        "target_detected_fraction": (round(sum(detected) / len(detected), 4)
                                     if detected else None),
        "target_lost_count": max(lost) if lost else 0,
        "collision_count": count_collisions(samples, collision_threshold),
        # --- 追加項目 ---
        "samples": len(samples),
        "target_redetect_count": max(redetect) if redetect else 0,
        "min_follow_distance_m": round(min(true_d), 4) if true_d else None,
        "max_follow_distance_m": round(max(true_d), 4) if true_d else None,
        "estimation_error_mae_m": (round(sum(est_err) / len(est_err), 4)
                                   if est_err else None),
        "leader_path_m": round(path_length(samples, "leader_x", "leader_y"), 3),
        "follower_path_m": round(path_length(samples, "follower_x", "follower_y"), 3),
        "state_histogram": histogram,
        "collision_threshold_m": collision_threshold,
    }
    return summary


def path_length(samples: list[dict], key_x: str, key_y: str) -> float:
    """走行距離。座標が欠けたサンプルは飛ばす。"""
    total = 0.0
    previous = None
    for s in samples:
        x, y = s.get(key_x), s.get(key_y)
        if x is None or y is None:
            continue
        if previous is not None:
            total += math.hypot(x - previous[0], y - previous[1])
        previous = (x, y)
    return total
