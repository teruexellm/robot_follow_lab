"""追従制御の判断部分。ROS の実行時 (rclpy) に依存しない。

`follow_controller.py` の `FollowController` から切り出してある。
狙いは 2 つ:

* **テストできるようにする** — 時刻や距離を引数で注入すれば、
  Gazebo も rclpy も無しに判断の正しさを確かめられる
  (`test/test_target_state.py`)。
* **改変しやすくする** — 追従の挙動を変えたい人が読む必要があるのは
  ほぼこのファイルだけで済む。ノード側 (購読・TF・action client) は
  触らなくてよい。

`followbot_interfaces.msg.TargetState` だけは import する。
状態の定数を再定義すると `.msg` とずれるため。
メッセージ定義を読むだけで、ROS のノードは起動しない。
"""

from __future__ import annotations

import math

from followbot_interfaces.msg import TargetState


STATE_LABELS = {
    TargetState.INIT: "INIT",
    TargetState.DETECTED: "DETECTED",
    TargetState.LOST: "LOST",
    TargetState.SEARCHING: "SEARCHING",
}


def classify_target_state(elapsed: float | None, lost_timeout: float,
                          search_timeout: float) -> int:
    """最後の検出からの経過時間から状態を決める (spec §43)。

    ROS に依存しない純粋関数として切り出してある。時刻を注入して
    unit test できるようにするため (spec §63 の "target lost timeout")。

        elapsed is None        一度も検出していない -> INIT
        elapsed <= lost_timeout      -> DETECTED
        elapsed <= search_timeout    -> LOST    (最後の有効位置を使い続ける)
        それ以上                      -> SEARCHING
    """
    if elapsed is None:
        return TargetState.INIT
    if elapsed <= lost_timeout:
        return TargetState.DETECTED
    if elapsed <= search_timeout:
        return TargetState.LOST
    return TargetState.SEARCHING


def estimate_target_velocity(history: list[tuple[float, float, float]],
                             window: float = 1.0,
                             min_samples: int = 3) -> tuple[float, float] | None:
    """target_pose の履歴から対象の速度 [m/s] を推定する。

    history は (時刻, x, y) の列で、時刻の昇順。末尾が最新。
    直近 window 秒に入る最古のサンプルと最新サンプルの差分で求める。
    最小二乗より素朴だが、推定値の意味が明確でテストしやすい。

    ROS に依存しない純粋関数にしてある (spec §63)。
    """
    if len(history) < min_samples:
        return None
    t_new, x_new, y_new = history[-1]
    oldest = None
    for sample in history:
        if t_new - sample[0] <= window:
            oldest = sample
            break
    if oldest is None:
        return None
    dt = t_new - oldest[0]
    if dt <= 1e-3:
        return None
    return (x_new - oldest[1]) / dt, (y_new - oldest[2]) / dt


def extrapolate_target(x: float, y: float, velocity: tuple[float, float] | None,
                       elapsed: float, max_lead: float
                       ) -> tuple[float, float]:
    """最後に見た位置を速度で前方へ外挿する (spec §17/§18 の範囲内)。

    使うのは自機センサ由来の target_pose の履歴だけで、Leader の odom や
    waypoint は一切見ない。外挿距離は max_lead で打ち切る。
    壁の向こうへ外挿して Nav2 の計画が失敗するのを防ぐため
    (幾何を直接知らずに済ませるには距離で頭打ちにするのが確実)。
    """
    if velocity is None:
        return x, y
    dx, dy = velocity[0] * elapsed, velocity[1] * elapsed
    lead = math.hypot(dx, dy)
    if lead < 1e-6:
        return x, y
    if lead > max_lead:
        dx *= max_lead / lead
        dy *= max_lead / lead
    return x + dx, y + dy


def should_rotate_in_place(goal_distance: float | None,
                           arrival_distance: float,
                           consecutive_aborts: int,
                           max_aborts: int) -> bool:
    """SEARCHING で前進するか定位置回転するかを決める。

    判断材料は 2 つ。

    1. ゴールが目前なら回転する (T-2)
    2. **ゴールへ行けないなら回転する** (T-7)
       Nav2 がゴールを連続で abort している場合、前進を選び続けても
       車体は動かない。動かなければ視野も変わらず再検出できない。
       外挿した点が地図の外に出た場合 (planner_server: worldToMap failed)
       に実際この状態へ落ちた。

    ROS に依存しない純粋関数にしてある (spec §63)。
    """
    if consecutive_aborts >= max_aborts:
        return True
    if goal_distance is None:
        return True
    return goal_distance <= arrival_distance


def is_redetection(previous: int, current: int) -> bool:
    """LOST / SEARCHING から DETECTED へ戻ったか (spec §44 の TARGET_REDETECTED)。"""
    return (current == TargetState.DETECTED
            and previous in (TargetState.LOST, TargetState.SEARCHING))
