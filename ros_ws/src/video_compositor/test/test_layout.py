"""動画のレイアウトとオーバレイの unit test (spec v2.0 §58, §59, §63)。

spec §63 の "video compositor" に対応する。OpenCV も ffmpeg も
Gazebo も起動せずに走る。

    pytest ros_ws/src/video_compositor/test/test_layout.py
"""

import pytest

from video_compositor.layout import (FPS, HEIGHT, WIDTH, bbox_to_pixels,
                                     fit_rect, nearest_index, overlay_lines,
                                     panel_rects)


# ---------- spec §57 の目標値 ----------


def test_target_resolution_and_fps():
    assert (WIDTH, HEIGHT) == (1280, 720)
    assert FPS == 10


# ---------- spec §58 のレイアウト ----------


def test_panels_cover_the_canvas_without_gaps():
    """4 つのパネルで 1280x720 を隙間も重なりも無く埋めること。"""
    rects = panel_rects()
    covered = 0
    for _, (x, y, w, h) in rects.items():
        assert x >= 0 and y >= 0
        assert x + w <= WIDTH, "右端を超えている"
        assert y + h <= HEIGHT, "下端を超えている"
        covered += w * h
    assert covered == WIDTH * HEIGHT


def test_panel_arrangement_matches_spec58():
    """左に overview、右列に上から leader / follower / telemetry。"""
    r = panel_rects()
    ox, oy, ow, oh = r["overview"]
    assert (ox, oy) == (0, 0)
    assert oh == HEIGHT, "overview は左列の全高を使う"

    # 右列は 3 つとも同じ x から始まり、同じ幅
    xs = {r[k][0] for k in ("leader", "follower", "telemetry")}
    ws = {r[k][2] for k in ("leader", "follower", "telemetry")}
    assert len(xs) == 1 and len(ws) == 1
    assert xs.pop() == ow, "右列は overview の直後から始まる"

    # 上から leader -> follower -> telemetry の順
    assert r["leader"][1] < r["follower"][1] < r["telemetry"][1]
    assert r["leader"][1] + r["leader"][3] == r["follower"][1]
    assert r["follower"][1] + r["follower"][3] == r["telemetry"][1]


def test_telemetry_panel_fits_all_spec59_items():
    """spec §59 の 7 項目が telemetry パネルに収まること。

    1 行 19 px で描くので、項目数 x 19 + 見出しぶんが高さ以内に
    収まっていないと下が切れる。
    """
    _, _, _, h = panel_rects()["telemetry"]
    needed = 40 + len(overlay_lines({}, "ai")) * 19
    assert needed <= h, f"telemetry パネルが {needed - h} px 足りない"


# ---------- アスペクト比の保持 ----------


def test_fit_rect_keeps_aspect_ratio():
    # 320x240 (4:3) を 432x270 (1.6:1) の枠へ入れる -> 高さ基準
    ox, oy, w, h = fit_rect(320, 240, 432, 270)
    assert (w, h) == (360, 270)
    assert ox == (432 - 360) // 2
    assert oy == 0
    assert w / h == pytest.approx(320 / 240)


def test_fit_rect_letterboxes_wide_source():
    ox, oy, w, h = fit_rect(800, 200, 400, 400)
    assert (w, h) == (400, 100)
    assert ox == 0
    assert oy == 150


def test_fit_rect_never_exceeds_the_frame():
    for src in ((1920, 1080), (100, 900), (1, 1), (800, 720)):
        ox, oy, w, h = fit_rect(src[0], src[1], 432, 270)
        assert ox >= 0 and oy >= 0
        assert ox + w <= 432
        assert oy + h <= 270


def test_fit_rect_handles_degenerate_source():
    # 0 幅の画像が来ても例外にせず枠いっぱいを返す
    assert fit_rect(0, 240, 432, 270) == (0, 0, 432, 270)


# ---------- bbox の画素変換 (spec §59) ----------


def test_bbox_center_of_frame():
    x1, y1, x2, y2 = bbox_to_pixels(0.5, 0.5, 0.5, 0.5, 320, 240)
    assert (x1, y1, x2, y2) == (80, 60, 240, 180)


def test_bbox_is_clipped_to_the_frame():
    """視野端の検出で bbox が画像外へ出ても枠内に丸めること。

    丸めないと OpenCV が描かないか例外になる。
    """
    x1, y1, x2, y2 = bbox_to_pixels(0.02, 0.5, 0.4, 0.4, 320, 240)
    assert x1 == 0
    assert 0 <= x2 <= 319
    assert x1 < x2

    x1, y1, x2, y2 = bbox_to_pixels(0.98, 0.98, 0.4, 0.4, 320, 240)
    assert x2 == 319
    assert y2 == 239


def test_bbox_scales_with_panel_size():
    """パネルに収めたサイズで変換すること。

    原寸 320x240 ではなく、実際に描いた 360x270 を渡す。
    原寸で計算すると bbox が映像とずれる。
    """
    small = bbox_to_pixels(0.5, 0.5, 0.2, 0.2, 320, 240)
    large = bbox_to_pixels(0.5, 0.5, 0.2, 0.2, 360, 270)
    assert large[2] - large[0] > small[2] - small[0]


# ---------- オーバレイ (spec §59) ----------


def test_overlay_has_all_spec59_items_in_order():
    labels = [label for label, _ in overlay_lines({}, "ai")]
    assert labels == ["SIM TIME", "PERCEPTION", "STATE", "CONFIDENCE",
                      "EST DIST", "TRUE DIST", "NAV2"]


def test_overlay_formats_values():
    row = {
        "sim_time": "123.456", "detected": "1", "target_state": "DETECTED",
        "confidence": "0.987", "estimated_distance_m": "1.234",
        "true_distance_m": "1.111", "nav2_status": "FollowPath:RUNNING",
    }
    values = dict(overlay_lines(row, "ai"))
    assert values["SIM TIME"] == "123.5 s"
    assert values["PERCEPTION"] == "AI"
    assert values["STATE"] == "DETECTED"
    assert values["CONFIDENCE"] == "0.99"
    assert values["EST DIST"] == "1.23 m"
    assert values["TRUE DIST"] == "1.11 m"
    assert values["NAV2"] == "FollowPath:RUNNING"


def test_overlay_shows_dash_for_missing_values():
    """列が欠けた行でも例外を出さずに "-" を出すこと。

    動画生成が途中で落ちるのが一番困る。検出していない行では
    confidence や推定距離が空欄になる。
    """
    values = dict(overlay_lines({"sim_time": "10.0", "detected": "0"}, "marker"))
    assert values["CONFIDENCE"] == "-"
    assert values["EST DIST"] == "-"
    assert values["STATE"] == "-"
    assert values["PERCEPTION"] == "MARKER"


def test_overlay_tolerates_garbage_values():
    values = dict(overlay_lines({"confidence": "NaN値", "sim_time": None}, "ai"))
    assert values["CONFIDENCE"] == "NaN値"
    assert values["SIM TIME"] == "-"


def test_overlay_uses_state_when_detected_flag_only():
    values = dict(overlay_lines({"detected": "1"}, "ai"))
    assert values["STATE"] == "DETECTED"


# ---------- 時刻の突き合わせ ----------


def test_nearest_index_picks_closest():
    times = [0.0, 1.0, 2.0, 3.0]
    assert nearest_index(times, 0.4) == 0
    assert nearest_index(times, 0.6) == 1
    assert nearest_index(times, 2.49) == 2
    assert nearest_index(times, 2.51) == 3


def test_nearest_index_clamps_outside_the_range():
    times = [10.0, 11.0, 12.0]
    assert nearest_index(times, 0.0) == 0
    assert nearest_index(times, 99.0) == 2


def test_nearest_index_exact_match():
    times = [0.0, 0.1, 0.2, 0.3, 0.4]
    for i, t in enumerate(times):
        assert nearest_index(times, t) == i


def test_nearest_index_single_element():
    assert nearest_index([5.0], 0.0) == 0
    assert nearest_index([5.0], 100.0) == 0


def test_nearest_index_rejects_empty():
    with pytest.raises(ValueError):
        nearest_index([], 1.0)


def test_nearest_index_on_uneven_spacing():
    """カメラは 10 Hz だが取りこぼすと間隔が開く。等間隔を仮定しないこと。"""
    times = [0.0, 0.1, 0.2, 5.0, 5.1]
    assert nearest_index(times, 2.0) == 2
    assert nearest_index(times, 4.0) == 3
