"""marker detector の unit test (spec v2.0 §63)。

合成画像で検証する。ROS も Gazebo も不要。

    pytest ros_ws/src/follower_perception/test/test_marker_detector.py
"""

import cv2
import numpy as np
import pytest

from follower_perception.marker_detector import (
    HsvRange,
    MarkerDetectorParams,
    build_mask,
    detect_marker,
)

W, H = 320, 240

# Leader と同じ青 (RGB)。Gazebo の diffuse 0.1 0.2 1.0 に相当
BLUE = (13, 38, 255)
# Follower の赤。誤検出してはならない
RED = (242, 26, 26)
# 床・壁のグレー
GRAY = (140, 140, 140)
# 障害物の茶色
BROWN = (153, 115, 77)


def make_image(background=GRAY) -> np.ndarray:
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:, :] = background
    return img


def draw_rect(img, x, y, w, h, color=BLUE):
    cv2.rectangle(img, (x, y), (x + w - 1, y + h - 1), color, thickness=-1)
    return img


def test_detects_centered_blue_rectangle():
    img = draw_rect(make_image(), 140, 100, 40, 40)
    det = detect_marker(img)
    assert det is not None
    assert det.center_x == pytest.approx((140 + 20) / W, abs=0.01)
    assert det.center_y == pytest.approx((100 + 20) / H, abs=0.01)
    assert det.width == pytest.approx(40 / W, abs=0.02)
    assert det.height == pytest.approx(40 / H, abs=0.02)
    assert det.fill_ratio > 0.9          # 矩形なので充填率は高い
    assert det.area_px >= 36


def test_returns_none_on_empty_scene():
    """Leader が写っていない画像では検出なし (spec §27 の negative)。"""
    assert detect_marker(make_image()) is None


def test_does_not_detect_red_follower():
    """赤 (Follower 自身の色) を青として誤検出しないこと。"""
    img = draw_rect(make_image(), 100, 80, 80, 80, color=RED)
    assert detect_marker(img) is None


def test_does_not_detect_brown_obstacle():
    img = draw_rect(make_image(), 100, 80, 90, 90, color=BROWN)
    assert detect_marker(img) is None


def test_picks_the_largest_blob():
    """MVP では Leader は 1 台。最大領域を採る。"""
    img = make_image()
    draw_rect(img, 20, 20, 20, 20)        # 小 400 px
    draw_rect(img, 200, 120, 60, 60)      # 大 3600 px
    det = detect_marker(img)
    assert det is not None
    assert det.center_x == pytest.approx((200 + 30) / W, abs=0.02)
    assert det.area_px > 3000


def test_small_speckles_are_removed_by_morphology():
    """1 px の青ノイズは open で消え、検出なしになること。"""
    img = make_image()
    rng = np.random.default_rng(42)
    for _ in range(60):
        x = int(rng.integers(0, W))
        y = int(rng.integers(0, H))
        img[y, x] = BLUE
    assert detect_marker(img) is None


def test_speckles_do_not_hide_the_real_target():
    img = draw_rect(make_image(), 150, 110, 40, 40)
    rng = np.random.default_rng(7)
    for _ in range(80):
        img[int(rng.integers(0, H)), int(rng.integers(0, W))] = BLUE
    det = detect_marker(img)
    assert det is not None
    assert det.center_x == pytest.approx(170 / W, abs=0.03)


def test_min_area_threshold_rejects_tiny_target():
    """min_area_px より小さい blob は切る。

    しきい値はパラメータで明示する。既定値は実測で決めており
    (DEVLOG T-6)、変わってもこのテストの意図は変わらないようにする。
    """
    img = draw_rect(make_image(), 160, 120, 5, 5)       # 25 px -> open 後 21 px
    assert detect_marker(img, MarkerDetectorParams(min_area_px=36)) is None
    assert detect_marker(img, MarkerDetectorParams(min_area_px=8)) is not None


def test_default_min_area_detects_distant_target():
    """既定値は遠方の小さな Leader を取りこぼさないこと (DEVLOG T-6)。

    既定が 36 px だった頃は demo_world で 3.98 m より遠い Leader を
    検出できず、見失ったあと再捕捉できずに追従が破綻していた
    (occlusion_test で検出率 0.277 / follow RMSE 3.70 m)。
    data/demo_eval の実測では、7.5 m の Leader の青領域は 26-40 px、
    morphology の open を通すとさらに減る。
    """
    # 4x4 = 16 px は open (3x3) を通すと 12 px まで減る。既定ではこれを通す
    assert detect_marker(draw_rect(make_image(), 160, 120, 4, 4)) is not None
    # ノイズ相当の極小 blob は既定でも落ちる
    assert detect_marker(draw_rect(make_image(), 160, 120, 3, 3)) is None
    assert detect_marker(draw_rect(make_image(), 160, 120, 2, 2)) is None


def test_detection_grows_as_target_approaches():
    near = detect_marker(draw_rect(make_image(), 110, 70, 100, 100))
    far = detect_marker(draw_rect(make_image(), 150, 110, 20, 20))
    assert near.width > far.width * 4
    assert near.area_px > far.area_px * 10


def test_bbox_stays_normalized_at_image_edge():
    """画像端にかかっても正規化範囲を外れないこと。"""
    for x in (-0, 0, 280):
        img = draw_rect(make_image(), x, 100, 60, 60)
        det = detect_marker(img)
        assert det is not None
        for value in det.as_tuple():
            assert 0.0 <= value <= 1.0, f"x={x} で範囲外: {det}"


def test_center_x_tracks_horizontal_position():
    """center_x が画像内の位置に単調に追従すること (方位計算の前提)。"""
    centers = []
    for x in (20, 90, 150, 220, 270):
        det = detect_marker(draw_rect(make_image(), x, 110, 30, 30))
        assert det is not None
        centers.append(det.center_x)
    assert centers == sorted(centers)
    assert centers[0] < 0.2 and centers[-1] > 0.8


def test_hsv_range_wrapping_hue():
    """H が 0 をまたぐ色 (赤) も 2 区間で扱えること。"""
    img = draw_rect(make_image(), 140, 100, 40, 40, color=RED)
    red_range = HsvRange(h_min=170, h_max=10, s_min=120, v_min=50)
    det = detect_marker(img, MarkerDetectorParams(hsv=red_range))
    assert det is not None
    assert det.center_x == pytest.approx(160 / W, abs=0.02)


def test_build_mask_rejects_bad_shape():
    with pytest.raises(ValueError):
        build_mask(np.zeros((H, W), dtype=np.uint8), MarkerDetectorParams())


def test_mask_is_binary():
    img = draw_rect(make_image(), 140, 100, 40, 40)
    mask = build_mask(img, MarkerDetectorParams())
    assert set(np.unique(mask)).issubset({0, 255})


def test_dark_scene_is_not_detected():
    """照明が落ちて V が低い場合は検出しない (v_min より暗い)。"""
    img = make_image(background=(0, 0, 0))
    draw_rect(img, 140, 100, 40, 40, color=(2, 5, 30))
    assert detect_marker(img) is None
