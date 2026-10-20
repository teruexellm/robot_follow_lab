"""カメラ投影と方位計算の unit test (spec v2.0 §63)。

ROS を起動せずに実行できる。

    pytest ros_ws/src/follower_perception/test/test_projection.py
"""

import math

import pytest

from follower_perception.projection import (
    CameraModel,
    bbox_from_points,
    box_corners,
    quaternion_to_matrix,
)

# FollowBot のカメラ (spec §13): 320x240 / FOV 約 100 度
CAM = CameraModel.from_fov(320, 240, 1.745)


def test_from_fov_round_trips():
    assert CAM.horizontal_fov == pytest.approx(1.745, abs=1e-6)
    # fx = (w/2) / tan(fov/2)
    assert CAM.fx == pytest.approx(160.0 / math.tan(1.745 / 2.0))
    assert CAM.fx == CAM.fy          # 正方画素


def test_from_fov_rejects_bad_input():
    with pytest.raises(ValueError):
        CameraModel.from_fov(0, 240, 1.0)
    with pytest.raises(ValueError):
        CameraModel.from_fov(320, 240, 0.0)
    with pytest.raises(ValueError):
        CameraModel.from_fov(320, 240, math.pi)


def test_from_camera_info():
    k = [100.0, 0.0, 160.0, 0.0, 100.0, 120.0, 0.0, 0.0, 1.0]
    cam = CameraModel.from_camera_info(320, 240, k)
    assert (cam.fx, cam.fy, cam.cx, cam.cy) == (100.0, 100.0, 160.0, 120.0)
    with pytest.raises(ValueError):
        CameraModel.from_camera_info(320, 240, [1.0] * 8)
    with pytest.raises(ValueError):
        CameraModel.from_camera_info(320, 240, [0.0] * 9)


def test_project_center_and_behind():
    # 光軸上の点は主点へ投影される
    u, v = CAM.project((0.0, 0.0, 2.0))
    assert u == pytest.approx(CAM.cx)
    assert v == pytest.approx(CAM.cy)
    # カメラ後方の点は投影できない
    assert CAM.project((0.0, 0.0, -1.0)) is None
    assert CAM.project((0.0, 0.0, 0.0)) is None


def test_project_is_scale_invariant_along_ray():
    near = CAM.project((0.3, 0.1, 1.0))
    far = CAM.project((0.9, 0.3, 3.0))
    assert near[0] == pytest.approx(far[0])
    assert near[1] == pytest.approx(far[1])


def test_bearing_sign_follows_rep103():
    """左が正、右が負 (REP-103)。光学フレームの x は右なので反転が必要。"""
    assert CAM.bearing_from_u(CAM.cx) == pytest.approx(0.0)
    # 画像の右側 (u 大) は方位が負
    assert CAM.bearing_from_u(CAM.cx + 80) < 0.0
    # 画像の左側 (u 小) は方位が正
    assert CAM.bearing_from_u(CAM.cx - 80) > 0.0


def test_bearing_at_image_edge_is_half_fov():
    left = CAM.bearing_from_u(0.0)
    right = CAM.bearing_from_u(float(CAM.width))
    assert left == pytest.approx(CAM.horizontal_fov / 2.0, abs=0.01)
    assert right == pytest.approx(-CAM.horizontal_fov / 2.0, abs=0.01)


def test_bearing_from_normalized_x_matches_pixel_version():
    """TargetDetection の center_x は正規化値 (spec §23, §37)。"""
    assert CAM.bearing_from_normalized_x(0.5) == pytest.approx(
        CAM.bearing_from_u(160.0))
    assert CAM.bearing_from_normalized_x(0.25) == pytest.approx(
        CAM.bearing_from_u(80.0))


def test_bearing_round_trip_through_projection():
    """3D 点 -> 投影 -> 方位 が真の方位と一致すること。"""
    for true_bearing in (-0.6, -0.2, 0.0, 0.3, 0.7):
        distance = 2.5
        # 光学フレーム: x が右なので左正の方位は -sin
        point = (-distance * math.sin(true_bearing), 0.0,
                 distance * math.cos(true_bearing))
        u, _ = CAM.project(point)
        assert CAM.bearing_from_u(u) == pytest.approx(true_bearing, abs=1e-6)


def test_elevation_sign():
    assert CAM.elevation_from_v(CAM.cy) == pytest.approx(0.0)
    assert CAM.elevation_from_v(CAM.cy + 50) < 0.0     # 画像下方 = 下向き


def test_quaternion_identity_and_90deg():
    identity = quaternion_to_matrix((0.0, 0.0, 0.0, 1.0))
    assert identity[0][0] == pytest.approx(1.0)
    assert identity[1][1] == pytest.approx(1.0)
    assert identity[2][2] == pytest.approx(1.0)

    # z 軸 90 度回転: x -> y
    rot = quaternion_to_matrix((0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
    assert rot[0][0] == pytest.approx(0.0, abs=1e-9)
    assert rot[1][0] == pytest.approx(1.0, abs=1e-9)

    with pytest.raises(ValueError):
        quaternion_to_matrix((0.0, 0.0, 0.0, 0.0))


def test_box_corners_count_and_extent():
    corners = box_corners((0.0, 0.0, 2.0), (0.4, 0.3, 0.2))
    assert len(corners) == 8
    assert min(c[0] for c in corners) == pytest.approx(-0.2)
    assert max(c[2] for c in corners) == pytest.approx(2.1)


def test_bbox_of_centered_box():
    """正面 2 m の箱が画像中央に、妥当な大きさで写ること。"""
    corners = box_corners((0.0, 0.0, 2.0), (0.35, 0.28, 0.15))
    bbox = bbox_from_points(CAM, corners)
    assert bbox is not None
    assert bbox.center_x == pytest.approx(0.5, abs=0.02)
    assert bbox.center_y == pytest.approx(0.5, abs=0.05)
    assert 0.0 < bbox.width < 0.5
    assert 0.0 < bbox.height < 0.5
    assert bbox.visible_ratio == pytest.approx(1.0, abs=1e-6)


def test_bbox_grows_as_target_approaches():
    far = bbox_from_points(CAM, box_corners((0.0, 0.0, 4.0), (0.35, 0.28, 0.15)))
    near = bbox_from_points(CAM, box_corners((0.0, 0.0, 1.0), (0.35, 0.28, 0.15)))
    assert near.width > far.width * 3


def test_bbox_is_none_when_behind_camera():
    corners = box_corners((0.0, 0.0, -2.0), (0.35, 0.28, 0.15))
    assert bbox_from_points(CAM, corners) is None


def test_bbox_is_none_when_far_off_axis():
    # 真横 (光学フレームでは x 方向) にあり、z が小さい -> 画面外
    corners = box_corners((5.0, 0.0, 0.5), (0.35, 0.28, 0.15))
    assert bbox_from_points(CAM, corners) is None


def test_partially_visible_box_is_clipped_and_reports_ratio():
    """画像端にかかる箱は bbox がクリップされ、visible_ratio が 1 未満になる。"""
    # 中心が左端付近に来るような位置
    corners = box_corners((-1.05, 0.0, 1.0), (0.6, 0.3, 0.2))
    bbox = bbox_from_points(CAM, corners, min_visible_ratio=0.0)
    assert bbox is not None
    assert bbox.visible_ratio < 1.0
    assert 0.0 <= bbox.center_x <= 1.0
    assert 0.0 <= bbox.center_x - bbox.width / 2 + 1e-9


def test_min_visible_ratio_suppresses_marginal_detection():
    """spec §27 の negative sample 判定に使えること。"""
    corners = box_corners((-1.15, 0.0, 1.0), (0.6, 0.3, 0.2))
    lenient = bbox_from_points(CAM, corners, min_visible_ratio=0.0)
    strict = bbox_from_points(CAM, corners, min_visible_ratio=0.9)
    assert lenient is not None
    assert strict is None


def test_bbox_values_stay_normalized():
    for z in (0.5, 1.0, 3.0, 7.0):
        bbox = bbox_from_points(CAM, box_corners((0.0, 0.0, z), (0.35, 0.28, 0.15)))
        if bbox is None:
            continue
        for value in bbox.as_tuple():
            assert 0.0 <= value <= 1.0, f"z={z} で正規化範囲外: {bbox}"
