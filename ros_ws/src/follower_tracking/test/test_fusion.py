"""LiDAR 融合の unit test (spec v2.0 §63)。

LaserScan を手で組み立てるため rclpy の起動は不要だが、
sensor_msgs の型定義は必要なので ROS 環境下で実行する。

    pytest ros_ws/src/follower_tracking/test/test_fusion.py
"""

import math

import pytest
from sensor_msgs.msg import LaserScan

from follower_tracking.target_pose_estimator import fuse_lidar_range

SAMPLES = 360


def make_scan(values_by_degree: dict[int, float],
              default: float = float("inf"),
              range_min: float = 0.1,
              range_max: float = 8.0) -> LaserScan:
    """度単位の指定から 360 点の LaserScan を作る (FollowBot と同じ構成)。"""
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = 2.0 * math.pi / SAMPLES
    scan.range_min = range_min
    scan.range_max = range_max
    ranges = [default] * SAMPLES
    for degree, value in values_by_degree.items():
        index = int(round((math.radians(degree) - scan.angle_min)
                          / scan.angle_increment)) % SAMPLES
        ranges[index] = value
    scan.ranges = ranges
    return scan


def test_returns_median_of_window():
    # 正面付近に 5 点。median は 2.00
    scan = make_scan({-2: 1.90, -1: 1.95, 0: 2.00, 1: 2.05, 2: 2.10})
    assert fuse_lidar_range(scan, 0.0, math.radians(6.0)) == pytest.approx(2.00)


def test_median_is_robust_against_outlier():
    """単一点ではなく median を使う理由 (spec §39)。"""
    scan = make_scan({-2: 2.00, -1: 2.01, 0: 7.90, 1: 1.99, 2: 2.00})
    value = fuse_lidar_range(scan, 0.0, math.radians(6.0))
    assert value == pytest.approx(2.00, abs=0.02)


def test_even_count_averages_middle_pair():
    scan = make_scan({0: 2.00, 1: 2.10})
    assert fuse_lidar_range(scan, 0.0, math.radians(2.0)) == pytest.approx(2.05)


def test_infinite_and_nan_values_are_ignored():
    scan = make_scan({-1: float("inf"), 0: 3.00, 1: float("nan"), 2: 3.02})
    assert fuse_lidar_range(scan, 0.0, math.radians(6.0)) == pytest.approx(3.01)


def test_values_outside_sensor_range_are_ignored():
    # range_max 以上 / range_min 以下は無効
    scan = make_scan({-1: 9.0, 0: 4.0, 1: 0.05, 2: 4.1})
    assert fuse_lidar_range(scan, 0.0, math.radians(6.0)) == pytest.approx(4.05)


def test_returns_none_when_too_few_valid_points():
    scan = make_scan({0: 2.0})
    assert fuse_lidar_range(scan, 0.0, math.radians(6.0), min_points=2) is None
    assert fuse_lidar_range(scan, 0.0, math.radians(6.0), min_points=1) == pytest.approx(2.0)


def test_returns_none_when_window_is_empty():
    scan = make_scan({90: 2.0, 91: 2.0})
    assert fuse_lidar_range(scan, 0.0, math.radians(6.0)) is None


def test_bearing_selects_the_right_window():
    """方位を変えると別の物体の距離が返ること。"""
    scan = make_scan({-1: 2.0, 0: 2.0, 1: 2.0,
                      89: 5.0, 90: 5.0, 91: 5.0})
    front = fuse_lidar_range(scan, 0.0, math.radians(4.0))
    left = fuse_lidar_range(scan, math.radians(90.0), math.radians(4.0))
    assert front == pytest.approx(2.0)
    assert left == pytest.approx(5.0)


def test_tolerance_controls_window_width():
    """窓幅そのものの検証。クラスタ選択は切って分離する。"""
    scan = make_scan({0: 2.0, 5: 6.0, -5: 6.0})
    narrow = fuse_lidar_range(scan, 0.0, math.radians(2.0), min_points=1,
                              cluster_width=0.0)
    wide = fuse_lidar_range(scan, 0.0, math.radians(8.0), min_points=1,
                            cluster_width=0.0)
    assert narrow == pytest.approx(2.0)
    # 広い窓では 2.0 / 6.0 / 6.0 の median = 6.0
    assert wide == pytest.approx(6.0)


def test_cluster_selection_prefers_near_object_over_wide_window():
    """同じ入力でも、クラスタ選択があれば近い物体を返す。"""
    scan = make_scan({0: 2.0, 5: 6.0, -5: 6.0})
    assert fuse_lidar_range(scan, 0.0, math.radians(8.0), min_points=1,
                            cluster_width=0.35) == pytest.approx(2.0)


def test_wraparound_at_pi_is_handled():
    """真後ろ (±180 度) をまたぐ窓でも角度差の正規化が効くこと。"""
    scan = make_scan({179: 3.0, -179: 3.1, 180: 3.05})
    value = fuse_lidar_range(scan, math.pi, math.radians(6.0))
    assert value is not None
    assert value == pytest.approx(3.05, abs=0.06)


def test_empty_scan_returns_none():
    scan = LaserScan()
    assert fuse_lidar_range(scan, 0.0, 0.1) is None


def test_zero_increment_returns_none():
    scan = make_scan({0: 2.0})
    scan.angle_increment = 0.0
    assert fuse_lidar_range(scan, 0.0, 0.1) is None


def test_min_valid_range_excludes_self_hits():
    """自機の車体・車輪のヒット (0.10-0.23 m) を除外できること (DEVLOG S-8)。

    クラスタ選択を入れると自機ヒットが「最も近いクラスタ」になるため、
    min_valid_range による除外は**より重要**になる。
    """
    scan = make_scan({-1: 0.11, 0: 0.22, 1: 2.00, 2: 2.02, 3: 2.01})
    without = fuse_lidar_range(scan, 0.0, math.radians(6.0))
    with_filter = fuse_lidar_range(scan, 0.0, math.radians(6.0), min_valid_range=0.30)
    # フィルタ無しでは自機ヒット (0.11, 0.22) が最近クラスタになり自機を返す
    assert without == pytest.approx(0.165, abs=0.01)
    # フィルタを入れると実物だけになる
    assert with_filter == pytest.approx(2.01)


def test_self_hits_dominate_nearest_cluster_without_filter():
    """自機ヒットは常に最も近いので、フィルタ無しでは必ず誤る。"""
    scan = make_scan({-2: 0.11, -1: 0.12, 0: 0.22, 1: 2.00, 2: 2.01})
    assert fuse_lidar_range(scan, 0.0, math.radians(6.0)) < 0.3
    assert fuse_lidar_range(scan, 0.0, math.radians(6.0),
                            min_valid_range=0.30) == pytest.approx(2.005)


def test_min_valid_range_can_make_result_none():
    scan = make_scan({0: 0.11, 1: 0.12})
    assert fuse_lidar_range(scan, 0.0, math.radians(6.0),
                            min_valid_range=0.30) is None


def test_nearest_cluster_is_selected_over_background_wall():
    """窓内に Leader と背後の壁が同時に入っても Leader を返すこと (DEVLOG X-2)。

    方位に誤差があると窓が広がり、壁の点が多数含まれる。
    単純な median では壁の距離になってしまう。
    """
    # Leader が 1.5 m に 3 点、壁が 4.0 m に 6 点
    scan = make_scan({-1: 1.50, 0: 1.52, 1: 1.51,
                      4: 4.00, 5: 4.02, 6: 4.01, 7: 3.99, 8: 4.03, 9: 4.00})
    plain = fuse_lidar_range(scan, 0.0, math.radians(12.0), cluster_width=0.0)
    clustered = fuse_lidar_range(scan, 0.0, math.radians(12.0), cluster_width=0.35)
    assert plain == pytest.approx(4.00, abs=0.05)       # 壁を返してしまう
    assert clustered == pytest.approx(1.51, abs=0.02)   # Leader を返す


def test_cluster_width_keeps_object_depth():
    """同一物体の奥行き (車体 0.35 m) はクラスタ内に収まること。"""
    scan = make_scan({-1: 1.50, 0: 1.60, 1: 1.70, 2: 1.80})
    value = fuse_lidar_range(scan, 0.0, math.radians(8.0), cluster_width=0.35)
    assert value == pytest.approx(1.65, abs=0.06)


def test_cluster_falls_back_when_too_few_points():
    """最近クラスタが min_points 未満なら窓全体で median を取る。"""
    scan = make_scan({0: 1.00, 4: 4.00, 5: 4.02, 6: 4.01})
    value = fuse_lidar_range(scan, 0.0, math.radians(12.0),
                             min_points=2, cluster_width=0.35)
    assert value == pytest.approx(4.01, abs=0.05)
