"""Target Lost / SEARCHING / REDETECTED の unit test (spec v2.0 §43, §44, §63)。

状態機械を ROS 非依存の純粋関数に切り出してあるため、時刻を注入して
境界条件まで検証できる。ROS や Gazebo の起動は不要。

    pytest ros_ws/src/follower_tracking/test/test_target_state.py
"""

import math

import pytest
from followbot_interfaces.msg import TargetState

from follower_tracking.follow_logic import (
    STATE_LABELS,
    classify_target_state,
    estimate_target_velocity,
    extrapolate_target,
    is_redetection,
    should_rotate_in_place,
)

# spec §43 の標準値
LOST = 0.5
SEARCH = 3.0


def state(elapsed):
    return classify_target_state(elapsed, LOST, SEARCH)


def test_never_detected_is_init():
    assert state(None) == TargetState.INIT


def test_fresh_detection_is_detected():
    assert state(0.0) == TargetState.DETECTED
    assert state(0.1) == TargetState.DETECTED
    assert state(0.49) == TargetState.DETECTED


def test_lost_timeout_boundary():
    """spec §43: 最後の有効検出から 0.5 秒以上で TARGET_LOST。"""
    assert state(LOST) == TargetState.DETECTED          # 境界は DETECTED 側
    assert state(LOST + 1e-6) == TargetState.LOST


def test_lost_holds_last_position_until_search_timeout():
    """spec §43: 短時間なら最後の有効位置を利用してよい。"""
    for elapsed in (0.6, 1.0, 2.0, 2.9):
        assert state(elapsed) == TargetState.LOST


def test_search_timeout_boundary():
    """spec §43: 2-3 秒以上見失ったら SEARCHING へ移行する。"""
    assert state(SEARCH) == TargetState.LOST            # 境界は LOST 側
    assert state(SEARCH + 1e-6) == TargetState.SEARCHING
    assert state(10.0) == TargetState.SEARCHING


def test_full_sequence_matches_spec_order():
    """spec §48 が要求する遷移順序 DETECTED -> LOST -> SEARCHING。"""
    sequence = [state(e) for e in (0.0, 0.3, 0.8, 2.5, 3.5, 8.0)]
    assert sequence == [
        TargetState.DETECTED, TargetState.DETECTED,
        TargetState.LOST, TargetState.LOST,
        TargetState.SEARCHING, TargetState.SEARCHING,
    ]


@pytest.mark.parametrize("previous, expected", [
    (TargetState.LOST, True),
    (TargetState.SEARCHING, True),
    (TargetState.INIT, False),
    (TargetState.DETECTED, False),
])
def test_redetection_only_from_lost_or_searching(previous, expected):
    """spec §44: 再検出時に TARGET_REDETECTED を生成する。

    INIT からの初回検出は「再検出」ではない。
    """
    assert is_redetection(previous, TargetState.DETECTED) is expected


def test_redetection_requires_detected_as_current():
    assert is_redetection(TargetState.LOST, TargetState.SEARCHING) is False
    assert is_redetection(TargetState.SEARCHING, TargetState.LOST) is False


def test_custom_timeouts_are_respected():
    """パラメータで閾値を変えられること (spec §43 は例示値)。"""
    assert classify_target_state(0.2, 0.1, 1.0) == TargetState.LOST
    assert classify_target_state(0.05, 0.1, 1.0) == TargetState.DETECTED
    assert classify_target_state(1.5, 0.1, 1.0) == TargetState.SEARCHING


def test_all_states_have_labels():
    """telemetry と video overlay が参照するラベルが全状態に存在すること。"""
    for value in (TargetState.INIT, TargetState.DETECTED,
                  TargetState.LOST, TargetState.SEARCHING):
        assert value in STATE_LABELS
        assert STATE_LABELS[value]


def test_label_names_match_spec_wording():
    """spec §43, §44 が使う名称と一致していること。"""
    assert STATE_LABELS[TargetState.DETECTED] == "DETECTED"
    assert STATE_LABELS[TargetState.LOST] == "LOST"
    assert STATE_LABELS[TargetState.SEARCHING] == "SEARCHING"


def test_search_switches_to_rotation_when_goal_reached():
    """SEARCHING の 2 段判定は「ゴールまでの距離」で行うこと (DEVLOG T-2)。

    残距離が追従距離と同程度なら、ゴールは目前であり向かっても動けない。
    「最後に見た位置までの距離」で判定すると、前進も回転もせず固まる。
    """
    follow_distance = 1.0
    arrival = 0.3

    def should_rotate(remaining):
        goal_distance = remaining - follow_distance
        return not (goal_distance > arrival)

    # 実測で固まった条件: 残距離 1.129 m / 追従距離 1.0 m
    assert should_rotate(1.129) is True      # ゴールは 0.129 m 先 -> 回転すべき
    assert should_rotate(1.2) is True        # 0.2 m 先 -> 回転
    assert should_rotate(1.4) is False       # 0.4 m 先 -> 向かう
    assert should_rotate(3.0) is False       # 2.0 m 先 -> 向かう
    # 旧実装 (残距離で判定) は 1.129 > 0.6 で「向かう」を選び固まっていた
    assert (1.129 > 0.6) is True


# ---------- 対象速度の推定と外挿 (spec §43 の「最後の有効位置の利用」) ----------


def test_velocity_needs_minimum_samples():
    assert estimate_target_velocity([]) is None
    assert estimate_target_velocity([(0.0, 0.0, 0.0)]) is None
    assert estimate_target_velocity([(0.0, 0.0, 0.0), (0.1, 0.1, 0.0)]) is None


def test_velocity_of_constant_motion():
    history = [(t / 10.0, 0.35 * t / 10.0, 0.0) for t in range(11)]
    vx, vy = estimate_target_velocity(history, window=1.0)
    assert vx == pytest.approx(0.35, abs=1e-6)
    assert vy == pytest.approx(0.0, abs=1e-6)


def test_velocity_has_both_components():
    history = [(t / 10.0, 0.2 * t / 10.0, -0.3 * t / 10.0) for t in range(11)]
    vx, vy = estimate_target_velocity(history, window=1.0)
    assert vx == pytest.approx(0.2, abs=1e-6)
    assert vy == pytest.approx(-0.3, abs=1e-6)


def test_velocity_of_stationary_target_is_zero():
    history = [(t / 10.0, 1.0, 2.0) for t in range(11)]
    assert estimate_target_velocity(history) == pytest.approx((0.0, 0.0))


def test_velocity_ignores_samples_older_than_window():
    # 10 s 静止したあと直近 1 s で動いた場合、直近だけを見る
    history = [(float(t), 0.0, 0.0) for t in range(10)]
    history += [(10.0 + t / 10.0, 0.5 * t / 10.0, 0.0) for t in range(1, 11)]
    vx, _ = estimate_target_velocity(history, window=1.0)
    assert vx == pytest.approx(0.5, abs=1e-6)


def test_velocity_needs_nonzero_time_span():
    assert estimate_target_velocity([(5.0, 1.0, 1.0)] * 3) is None


def test_extrapolate_without_velocity_keeps_position():
    assert extrapolate_target(1.0, 2.0, None, 5.0, 2.5) == (1.0, 2.0)


def test_extrapolate_moves_forward_by_elapsed_time():
    x, y = extrapolate_target(0.0, 0.0, (0.35, 0.0), 2.0, 2.5)
    assert x == pytest.approx(0.70)
    assert y == pytest.approx(0.0)


def test_extrapolate_is_capped_by_max_lead():
    # 0.35 m/s を 20 s 外挿すると 7 m になるが、上限 2.5 m に収める
    x, y = extrapolate_target(0.0, 0.0, (0.35, 0.0), 20.0, 2.5)
    assert math.hypot(x, y) == pytest.approx(2.5)
    assert x == pytest.approx(2.5)


def test_extrapolate_keeps_direction_when_capped():
    x, y = extrapolate_target(0.0, 0.0, (0.3, 0.4), 100.0, 1.0)
    assert math.hypot(x, y) == pytest.approx(1.0)
    assert (x, y) == pytest.approx((0.6, 0.8))


def test_extrapolate_with_zero_velocity_keeps_position():
    assert extrapolate_target(3.0, 4.0, (0.0, 0.0), 5.0, 2.5) == (3.0, 4.0)


def test_extrapolation_unsticks_target_lost_closer_than_follow_distance():
    """最終既知距離 1.0 m 未満で見失った場合に前進できること。

    最終既知位置のままだと goal_distance が負になり、前進も外挿もせず
    その場回転で固まる (実測: 0.58 m のまま 46 s 回転。DEVLOG T-5)。
    0.35 m/s で 2 s 外挿すれば 1.28 m になり、ゴールが前方に生まれる。
    """
    x, _ = extrapolate_target(0.58, 0.0, (0.35, 0.0), 2.0, 2.5)
    assert x == pytest.approx(1.28)
    assert x - 1.0 > 0.0


# ---------- SEARCHING で前進するか回転するか (spec §43) ----------

ARRIVAL = 0.3
MAX_ABORTS = 3


def test_drives_when_goal_is_far_and_reachable():
    assert should_rotate_in_place(1.5, ARRIVAL, 0, MAX_ABORTS) is False


def test_rotates_when_goal_is_already_reached():
    # ゴールが目前なら向かっても動かないので回転する (T-2)
    assert should_rotate_in_place(0.1, ARRIVAL, 0, MAX_ABORTS) is True


def test_arrival_boundary_is_inclusive():
    assert should_rotate_in_place(ARRIVAL, ARRIVAL, 0, MAX_ABORTS) is True
    assert should_rotate_in_place(ARRIVAL + 1e-6, ARRIVAL, 0, MAX_ABORTS) is False


def test_rotates_when_goal_distance_unknown():
    assert should_rotate_in_place(None, ARRIVAL, 0, MAX_ABORTS) is True


def test_rotates_when_goal_keeps_being_aborted():
    """到達できないゴールを追い続けないこと (T-7)。

    外挿した点が地図の外に出ると planner が worldToMap で失敗し、
    ゴールは abort され続ける。前進を選び続けると車体が動かず、
    視野も変わらないので永久に再検出できない状態に落ちる。
    """
    assert should_rotate_in_place(5.0, ARRIVAL, MAX_ABORTS, MAX_ABORTS) is True


def test_tolerates_aborts_below_threshold():
    # 追従中は preemption で abort が普通に起きるので、少数では切り替えない
    assert should_rotate_in_place(5.0, ARRIVAL, MAX_ABORTS - 1, MAX_ABORTS) is False


def test_negative_goal_distance_rotates():
    """見失った時点で対象が追従距離より近いと goal_distance は負になる。

    外挿 (T-5) が効かない場合でも、前進ではなく回転を選んで
    視野を変えにいく。
    """
    assert should_rotate_in_place(-0.42, ARRIVAL, 0, MAX_ABORTS) is True


def test_follow_controller経由でも同じ関数をimportできる():
    """follow_controller 経由の import を壊さない。

    判断部分は follow_logic.py へ移したが、`from
    follower_tracking.follow_controller import classify_target_state` と
    書いてあるコードは外にもありうる。再 export を外すと黙って壊れるので、
    同一オブジェクトであることをここで固定する。
    """
    from follower_tracking import follow_controller, follow_logic

    for name in ("STATE_LABELS", "classify_target_state",
                 "estimate_target_velocity", "extrapolate_target",
                 "is_redetection", "should_rotate_in_place"):
        assert getattr(follow_controller, name) is getattr(follow_logic, name)


def test_follow_logic_は_rclpy_を読み込まない():
    """判断部分だけを ROS 実行時なしでテストできる状態を保つ。

    ここに rclpy 依存が混ざると、この層のテストが Gazebo や
    ノード起動を要求するようになる。
    """
    import subprocess
    import sys

    out = subprocess.run(
        [sys.executable, "-c",
         "import sys; import follower_tracking.follow_logic;"
         " print('rclpy' in sys.modules)"],
        capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "False"
