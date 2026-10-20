"""telemetry / summary の算出の unit test (spec v2.0 §55, §56, §63)。

ROS も Gazebo も起動せずに走る。サンプル列を直接与えて境界条件まで見る。

    pytest ros_ws/src/experiment_logger/test/test_telemetry.py
"""

import math

import pytest

from experiment_logger.telemetry import (COLUMNS, EVENT_COLUMNS,
                                         count_collisions, distance,
                                         follow_error, path_length, summarize)

DESIRED = 1.0
COLLISION = 0.45


def make_sample(t, true_d, **extra):
    row = {c: None for c in COLUMNS}
    row.update({"sim_time": t, "true_distance_m": true_d})
    row.update(extra)
    return row


# ---------- 列の定義 ----------


def test_spec55_required_columns_come_first():
    """spec §55 の必須 10 項目が先頭に並んでいること。"""
    required = [
        "sim_time", "true_distance_m", "estimated_distance_m", "detected",
        "confidence", "follow_error_m", "follower_vx", "follower_wz",
        "nav2_status", "target_state",
    ]
    assert COLUMNS[:len(required)] == required


def test_event_columns():
    assert EVENT_COLUMNS == ["sim_time", "event", "detail"]


# ---------- 補助関数 ----------


def test_follow_error_sign():
    # 離れすぎなら正、近づきすぎなら負
    assert follow_error(1.5, DESIRED) == pytest.approx(0.5)
    assert follow_error(0.7, DESIRED) == pytest.approx(-0.3)
    assert follow_error(None, DESIRED) is None


def test_distance_handles_missing():
    assert distance((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)
    assert distance(None, (1.0, 1.0)) is None
    assert distance((1.0, 1.0), None) is None


def test_path_length_skips_missing_points():
    samples = [
        {"x": 0.0, "y": 0.0},
        {"x": None, "y": None},     # 欠けたサンプルは飛ばす
        {"x": 3.0, "y": 4.0},
    ]
    assert path_length(samples, "x", "y") == pytest.approx(5.0)


def test_path_length_of_single_point_is_zero():
    assert path_length([{"x": 1.0, "y": 2.0}], "x", "y") == 0.0


# ---------- 衝突の数え方 ----------


def test_collision_counts_intervals_not_samples():
    """1 回の接触を何十回も数えないこと。

    10 Hz で 2 秒接触すると 20 サンプルになる。サンプル数で数えると
    「衝突 20 回」になってしまう。
    """
    samples = [make_sample(i * 0.1, 0.3) for i in range(20)]
    assert count_collisions(samples, COLLISION) == 1


def test_collision_counts_separate_contacts():
    far = [make_sample(i * 0.1, 1.0) for i in range(5)]
    near = [make_sample(i * 0.1, 0.3) for i in range(5)]
    assert count_collisions(near + far + near, COLLISION) == 2


def test_no_collision_when_always_clear():
    samples = [make_sample(i * 0.1, 1.2) for i in range(10)]
    assert count_collisions(samples, COLLISION) == 0


def test_collision_threshold_boundary():
    # しきい値ちょうどは衝突としない (< で判定)
    assert count_collisions([make_sample(0.0, COLLISION)], COLLISION) == 0
    assert count_collisions([make_sample(0.0, COLLISION - 1e-6)], COLLISION) == 1


def test_collision_ignores_missing_distance():
    samples = [make_sample(0.0, None), make_sample(0.1, 0.3)]
    assert count_collisions(samples, COLLISION) == 1


# ---------- summary ----------


def summary_of(samples):
    return summarize(samples, scenario="occlusion_test", perception="ai",
                     desired=DESIRED, collision_threshold=COLLISION)


def test_summary_has_spec56_fields():
    """spec §56 の例が挙げる 9 項目をすべて含むこと。"""
    samples = [make_sample(i * 0.1, 1.0, detected=1, lost_count=0,
                           redetect_count=0, target_state="DETECTED")
               for i in range(10)]
    s = summary_of(samples)
    for key in ("scenario", "perception", "duration_sec",
                "desired_follow_distance_m", "mean_follow_distance_m",
                "follow_rmse_m", "target_detected_fraction",
                "target_lost_count", "collision_count"):
        assert key in s, key


def test_summary_of_empty_samples_does_not_crash():
    s = summary_of([])
    assert s["samples"] == 0
    assert s["duration_sec"] == 0.0


def test_rmse_is_zero_when_distance_matches_desired():
    samples = [make_sample(i * 0.1, DESIRED) for i in range(10)]
    assert summary_of(samples)["follow_rmse_m"] == pytest.approx(0.0)


def test_rmse_uses_desired_not_mean():
    """RMSE は平均からのばらつきではなく、**目標追従距離からの誤差**。

    常に 2.0 m で追従していれば分散は 0 だが、目標 1.0 m からは 1.0 m
    ずれているので RMSE は 1.0 でなければならない。
    """
    samples = [make_sample(i * 0.1, 2.0) for i in range(10)]
    s = summary_of(samples)
    assert s["follow_rmse_m"] == pytest.approx(1.0)
    assert s["mean_follow_distance_m"] == pytest.approx(2.0)


def test_rmse_value():
    samples = [make_sample(0.0, 0.5), make_sample(0.1, 1.5)]
    # 誤差 -0.5, +0.5 -> RMSE = 0.5
    assert summary_of(samples)["follow_rmse_m"] == pytest.approx(0.5)


def test_detected_fraction():
    samples = ([make_sample(i * 0.1, 1.0, detected=1) for i in range(3)]
               + [make_sample(0.4, 1.0, detected=0)])
    assert summary_of(samples)["target_detected_fraction"] == pytest.approx(0.75)


def test_counts_take_the_maximum_not_the_last():
    """lost_count は単調増加なので最大値を採る。

    最後のサンプルを採ると、計測終了時にたまたま状態メッセージが
    届いていない場合に 0 になってしまう。
    """
    samples = [make_sample(0.0, 1.0, lost_count=3, redetect_count=2),
               make_sample(0.1, 1.0, lost_count=None, redetect_count=None)]
    s = summary_of(samples)
    assert s["target_lost_count"] == 3
    assert s["target_redetect_count"] == 2


def test_state_histogram():
    samples = ([make_sample(i * 0.1, 1.0, target_state="DETECTED") for i in range(3)]
               + [make_sample(0.4, 1.0, target_state="LOST")])
    assert summary_of(samples)["state_histogram"] == {"DETECTED": 3, "LOST": 1}


def test_duration_is_span_of_samples():
    samples = [make_sample(10.0, 1.0), make_sample(310.0, 1.0)]
    assert summary_of(samples)["duration_sec"] == pytest.approx(300.0)


def test_path_lengths_are_reported():
    samples = [make_sample(0.0, 1.0, leader_x=0.0, leader_y=0.0,
                           follower_x=0.0, follower_y=0.0),
               make_sample(0.1, 1.0, leader_x=3.0, leader_y=4.0,
                           follower_x=1.0, follower_y=0.0)]
    s = summary_of(samples)
    assert s["leader_path_m"] == pytest.approx(5.0)
    assert s["follower_path_m"] == pytest.approx(1.0)


def test_estimation_error_is_mean_absolute():
    samples = [make_sample(0.0, 1.0, estimation_error_m=0.1),
               make_sample(0.1, 1.0, estimation_error_m=0.3)]
    assert summary_of(samples)["estimation_error_mae_m"] == pytest.approx(0.2)


def test_missing_optional_fields_do_not_crash():
    """検出も状態も一度も来なかった場合でも summary は作れること。"""
    samples = [make_sample(i * 0.1, 1.0) for i in range(5)]
    s = summary_of(samples)
    assert s["target_detected_fraction"] is None
    assert s["target_lost_count"] == 0
    assert s["state_histogram"] == {}
    assert math.isfinite(s["follow_rmse_m"])
